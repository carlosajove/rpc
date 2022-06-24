#include "controller/whole_body_controller/ihwbc/ihwbc.hpp"
#include "controller/whole_body_controller/contact.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/internal_constraint.hpp"
#include "controller/whole_body_controller/task.hpp"

IHWBC::IHWBC(const Eigen::MatrixXd &sa, const Eigen::MatrixXd *sf,
             const Eigen::MatrixXd *sv)
    : sa_(sa), dim_contact_(0), b_contact_(true), lambda_qddot_(0.),
      lambda_rf_(0.) {

  num_qdot_ = sa_.cols();

  if (!sf) {
    sf_ = *sf;
    b_floating_ = true;
    num_float_ = sf_.rows();
  } else {
    sf_.setZero();
    b_floating_ = false;
    num_float_ = 0;
  }

  if (!sv) {
    sv_ = *sv;
    b_passive_ = true;
    num_passive_ = sv_.rows();
  } else {
    sv_.setZero();
    b_passive_ = false;
    num_passive_ = 0;
  }

  A_.setZero(num_qdot_, num_qdot_);
  Ainv_.setZero(num_qdot_, num_qdot_);
  cori_.setZero(num_qdot_);
  grav_.setZero(num_qdot_);
  snf_.setZero(num_qdot_ - num_float_, num_qdot_);
  snf_.rightCols(num_qdot_ - num_float_) =
      Eigen::MatrixXd::Identity(num_qdot_ - num_float_, num_qdot_ - num_float_);
}

IHWBC::~IHWBC() {}

void IHWBC::UpdateSetting(const Eigen::MatrixXd &A, const Eigen::MatrixXd &Ainv,
                          const Eigen::VectorXd &cori,
                          const Eigen::VectorXd &grav) {
  A_ = A;
  Ainv_ = Ainv;
  cori_ = cori;
  grav_ = grav;
}

void IHWBC::Solve(
    const std::vector<Task *> &task_container,
    const std::vector<Contact *> &contact_container,
    const std::vector<InternalConstraint *> &internal_constraint_container,
    const std::vector<ForceTask *> &force_task_container,
    Eigen::VectorXd &qddot_cmd, Eigen::VectorXd &rf_cmd,
    Eigen::VectorXd &trq_cmd) {

  assert(task_container.size() > 0);
  b_contact_ = contact_container.size() > 0 ? true : false;

  //=============================================================
  // cost setup
  //=============================================================
  Eigen::MatrixXd cost_mat, cost_t_mat, cost_rf_mat;
  Eigen::VectorXd cost_vec, cost_t_vec, cost_rf_vec;

  // task cost
  cost_t_mat.setZero(num_qdot_, num_qdot_);
  cost_t_vec.setZero(num_qdot_);
  for (const auto &task : task_container) {
    Eigen::MatrixXd jt = task->Jacobian();
    Eigen::VectorXd jtdot_qdot = task->JacobianDotQdot();
    Eigen::VectorXd des_xddot = task->OscCommand();
    Eigen::MatrixXd weight_mat = task->Weight().asDiagonal();

    cost_t_mat += jt.transpose() * weight_mat * jt;
    cost_t_vec += (jtdot_qdot - des_xddot).transpose() * weight_mat * jt;
  }
  cost_t_mat += lambda_qddot_ * A_; // regularization term

  // // check contact dimension
  static bool b_first_visit(true);
  if (b_first_visit) {
    for (const auto &contact : contact_container)
      dim_contact_ += contact->Dim();
    b_first_visit = false;
  }

  cost_mat.setZero(num_qdot_ + dim_contact_, num_qdot_ + dim_contact_);
  cost_vec.setZero(num_qdot_ + dim_contact_);

  // reaction force cost
  if (b_contact_) {
    // exist contact
    cost_rf_mat.setZero(dim_contact_, dim_contact_);
    cost_rf_vec.setZero(dim_contact_);
    int row_idx(0);
    for (const auto &force_task : force_task_container) {
      // lfoot, rfoot order & wrench (torque, force order)
      Eigen::MatrixXd weight_mat = force_task->Weight().asDiagonal();
      Eigen::VectorXd desired_rf = force_task->DesiredRf();
      int dim = force_task->Dim();

      cost_rf_mat.block(row_idx, row_idx, dim, dim) = weight_mat;
      cost_rf_vec.segment(row_idx, dim) = desired_rf;
      row_idx += dim;
    }
    cost_rf_mat +=
        lambda_rf_ * Eigen::MatrixXd::Identity(dim_contact_, dim_contact_);

    cost_mat.topLeftCorner(cost_t_mat.rows(), cost_t_mat.cols()) = cost_t_mat;
    cost_mat.bottomRightCorner(cost_rf_mat.rows(), cost_rf_mat.cols()) =
        cost_rf_mat;
    cost_vec.head(cost_t_vec.size()) = cost_t_vec;
    cost_vec.tail(cost_rf_vec.size()) = cost_rf_vec;
  } else {
    // no contact
    cost_mat = cost_t_mat;
    cost_vec = cost_t_vec;
  }
  //=============================================================
  // equality constraint (Dynamics) setup
  //=============================================================

  // internal constraints setup
  Eigen::MatrixXd ji_mat, ni,
      sa_ni_trc_bar; // TODO: sa_ni_trc_bar using just pseudo inv (not
                     // dynamically consistent pseudo inv)
  Eigen::VectorXd jidot_qdot_vec, ji_mat_transpose_lambda_int_jidot_qdot_vec;
  if (b_passive_) {
    // exist passive joint

    ji_mat.setZero(num_passive_, num_qdot_);
    jidot_qdot_vec.setZero(num_passive_);
    // ni.setZero(num_qdot_, num_qdot_); //TODO:useless?
    ji_mat_transpose_lambda_int_jidot_qdot_vec.setZero(num_qdot_);

    int row_idx(0);
    for (const auto &int_constraint : internal_constraint_container) {
      Eigen::MatrixXd ji = int_constraint->Jacobian();
      Eigen::VectorXd jidot_qdot = int_constraint->JacobianDotQdot();
      int dim = int_constraint->Dim();

      ji_mat.middleRows(row_idx, dim) = ji;
      jidot_qdot_vec.segment(row_idx, dim) = jidot_qdot;
      row_idx += dim;
    }

    Eigen::MatrixXd ji_mat_bar =
        util::WeightedPseudoInverse(ji_mat, Ainv_, 0.0001);
    ni = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_) - ji_mat_bar * ji_mat;
    Eigen::MatrixXd lambda_int_inv = ji_mat * Ainv_ * ji_mat.transpose();
    ji_mat_transpose_lambda_int_jidot_qdot_vec =
        ji_mat.transpose() * util::PseudoInverse(lambda_int_inv, 0.0001) *
        jidot_qdot_vec;

    // compuationally efficient pseudo inverse for trq calc
    Eigen::MatrixXd sa_ni_trc = sa_ * ni.rightCols(num_active_ + num_passive_);
    Eigen::MatrixXd Ainv_trc = Ainv_.bottomRightCorner(
        num_active_ + num_passive_, num_active_ + num_passive_);
    Eigen::MatrixXd sa_ni_trc_bar =
        util::WeightedPseudoInverse(sa_ni_trc, Ainv_trc, 0.0001);
    // util::PseudoInverse(sa_ni_trc, 0.0001);

  } else {
    // no passive joint
    ni.setIdentity(num_qdot_, num_qdot_);
    ji_mat_transpose_lambda_int_jidot_qdot_vec.setZero(num_qdot_);
    sa_ni_trc_bar.setIdentity(num_active_, num_active_);
  }

  // contact setup
  Eigen::MatrixXd jc_mat, uf_mat;
  Eigen::VectorXd uf_vec;
  if (b_contact_) {
    // contact exist
    int row_idx(0);
    for (const auto &contact : contact_container) {
      Eigen::MatrixXd jc = contact->Jacobian();
      Eigen::MatrixXd cone_mat = contact->UfMatrix();
      Eigen::VectorXd cone_vec = contact->UfVector();
      int dim = contact->Dim();

      jc_mat.middleRows(row_idx, dim) = jc;
      uf_mat.middleRows(row_idx, dim) = cone_mat;
      uf_vec.segment(row_idx, dim) = cone_vec;
      row_idx += dim;
    }
  } else {
    // no contact
  }

  // equality constraints mat & vec
  Eigen::MatrixXd eq_mat, eq_float_mat, eq_int_mat;
  Eigen::VectorXd eq_vec, eq_float_vec, eq_int_vec;

  if (b_contact_) {
    if (b_floating_) {
      if (b_passive_) {
        // floating base: o, internal constraint: o, contact: o
        eq_float_mat.setZero(num_float_, num_qdot_ + dim_contact_);
        eq_float_vec.setZero(num_float_);

        eq_float_mat.leftCols(num_qdot_) = sf_ * A_;
        eq_float_mat.rightCols(dim_contact_) =
            -sf_ * ni.transpose() * jc_mat.transpose();
        eq_float_vec = sf_ * ni.transpose() * (cori_ + grav_);

        eq_int_mat.setZero(num_passive_, num_qdot_ + dim_contact_);
        eq_int_vec.setZero(num_passive_);

        eq_int_mat.leftCols(num_qdot_) = ji_mat;
        eq_int_vec = jidot_qdot_vec;

        eq_mat.setZero(num_float_ + num_passive_, num_qdot_ + dim_contact_);
        eq_vec.setZero(num_float_ + num_passive_);

        eq_mat.topRows(num_float_) = eq_float_mat;
        eq_mat.bottomRows(num_passive_) = eq_int_mat;
        eq_vec.head(num_float_) = eq_float_vec;
        eq_vec.tail(num_passive_) = eq_int_vec;

      } else {
        // floating base: o, internal constraint: x, contact: o
        eq_float_mat.setZero(num_float_, num_qdot_ + dim_contact_);
        eq_float_vec.setZero(num_float_);

        eq_float_mat.leftCols(num_qdot_) = sf_ * A_;
        eq_float_mat.rightCols(dim_contact_) =
            -sf_ * ni.transpose() * jc_mat.transpose();
        eq_float_vec = sf_ * ni.transpose() * (cori_ + grav_);

        eq_mat = eq_float_mat;
        eq_vec = eq_float_vec;
      }
    } else {
      if (b_passive_) {
        // floating base: x, internal constraint: o, contact: o
        eq_int_mat.setZero(num_passive_, num_qdot_ + dim_contact_);
        eq_int_mat.leftCols(num_qdot_) = ji_mat;
        eq_int_vec = jidot_qdot_vec;

        eq_mat = eq_int_mat;
        eq_vec = eq_int_vec;

      } else {
        // floating base: x, internal constraint: x, contact: o
        eq_mat.setZero(0, num_qdot_ + dim_contact_);
        eq_vec.setZero(0);
      }
    }
  } else {
    if (b_floating_) {
      if (b_passive_) {
        // floating base: o, internal constraint: o, contact: x
        eq_float_mat = sf_ * A_;
        eq_float_vec = sf_ * ni.transpose() * (cori_ + grav_);

        eq_int_mat = ji_mat;
        eq_int_vec = jidot_qdot_vec;

        eq_mat.setZero(num_float_ + num_passive_, num_qdot_);
        eq_vec.setZero(num_float_ + num_passive_);

        eq_mat.topRows(num_float_) = eq_float_mat;
        eq_mat.bottomRows(num_passive_) = eq_int_mat;
        eq_vec.head(num_float_) = eq_float_vec;
        eq_vec.tail(num_passive_) = eq_int_vec;
      } else {
        // floating base: o, internal contstraint: x, contact: x
        eq_float_mat = sf_ * A_;
        eq_float_vec = sf_ * ni.transpose() * (cori_ + grav_);

        eq_mat = eq_float_mat;
        eq_vec = eq_float_vec;
      }
    } else {
      if (b_passive_) {
        // floating base: x, internal constraint: o, contact: x
        eq_int_mat = ji_mat;
        eq_int_vec = jidot_qdot_vec;

        eq_mat = eq_int_mat;
        eq_vec = eq_int_vec;

      } else {
        // floating base: x, internal constraint: x, contact: x
        eq_mat.setZero(0, num_qdot_);
        eq_vec.setZero(0);
      }
    }
  }

  //=============================================================
  // inequality constraint setup
  //=============================================================
  // Uf >= fz
  Eigen::MatrixXd ineq_mat;
  Eigen::VectorXd ineq_vec;
  if (!b_trq_limit_) {
    if (b_contact_) {
      // trq limit : x, contact: o
      Eigen::MatrixXd ineq_contact_mat;
      Eigen::VectorXd ineq_contact_vec;
      int dim_cone_constraint = uf_mat.rows();

      ineq_contact_mat.setZero(dim_cone_constraint, num_qdot_ + dim_contact_);
      ineq_contact_vec.setZero(dim_cone_constraint);

      ineq_contact_mat.rightCols(dim_contact_) = uf_mat;
      ineq_contact_vec.tail(dim_cone_constraint) = -uf_vec;

      ineq_mat = ineq_contact_mat;
      ineq_vec = ineq_contact_vec;
    } else {
      // trq limit: x, contact: x
      ineq_mat.setZero(0, num_qdot_);
      ineq_vec.setZero(0);
    }
  } else {
    Eigen::MatrixXd ineq_trq_mat;
    Eigen::VectorXd ineq_trq_vec;

    Eigen::MatrixXd l_trq_mat, r_trq_mat;
    Eigen::VectorXd l_trq_vec, r_trq_vec;
    l_trq_mat.setZero(num_qdot_ - num_float_, num_qdot_ + dim_contact_);
    r_trq_mat.setZero(num_qdot_ - num_float_, num_qdot_ + dim_contact_);
    // l_trq_vec.setZero(num_qdot_ - num_float_);
    // r_trq_vec.setZero(num_qdot_ - num_float_);

    l_trq_mat.leftCols(num_qdot_) = sa_ni_trc_bar.transpose() * snf_ * A_;
    l_trq_mat.rightCols(dim_contact_) =
        -sa_ni_trc_bar.transpose() * snf_ * (jc_mat * ni).transpose();
    l_trq_vec =
        -joint_trq_limits_.leftCols(1) +
        sa_ni_trc_bar.transpose() * snf_ * ni.transpose() * (cori_ + grav_) +
        sa_ni_trc_bar.transpose() * snf_ *
            ji_mat_transpose_lambda_int_jidot_qdot_vec;

    r_trq_mat = l_trq_mat;
    r_trq_vec =
        joint_trq_limits_.rightCols(1) -
        sa_ni_trc_bar.transpose() * snf_ * ni.transpose() * (cori_ + grav_) -
        sa_ni_trc_bar.transpose() * snf_ *
            ji_mat_transpose_lambda_int_jidot_qdot_vec;

    ineq_trq_mat.setZero(2 * (num_qdot_ - num_float_), num_qdot_ - num_float_);
    ineq_trq_vec.setZero(2 * (num_qdot_ - num_float_));
    ineq_trq_mat.topRows(num_qdot_ - num_float_) = l_trq_mat;
    ineq_trq_mat.bottomRows(num_qdot_ - num_float_) = r_trq_mat;
    ineq_trq_vec.head(num_qdot_ - num_float_) = l_trq_vec;
    ineq_trq_vec.tail(num_qdot_ - num_float_) = r_trq_vec;

    if (b_contact_) {
      // trq limit: o, contact: o
      Eigen::MatrixXd ineq_contact_mat;
      Eigen::VectorXd ineq_contact_vec;
      int dim_cone_constraint = uf_mat.rows();

      ineq_contact_mat.setZero(dim_cone_constraint, num_qdot_ + dim_contact_);
      ineq_contact_vec.setZero(dim_cone_constraint);

      ineq_contact_mat.rightCols(dim_contact_) = uf_mat;
      ineq_contact_vec.tail(dim_cone_constraint) = -uf_vec;

      ineq_mat.setZero(ineq_trq_mat.rows() + ineq_contact_mat.rows(),
                       num_qdot_ + dim_contact_);
      ineq_vec.setZero(ineq_trq_mat.rows() + ineq_contact_mat.rows());

      ineq_mat.topRows(ineq_trq_mat.rows()) = ineq_trq_mat;
      ineq_mat.bottomRows(ineq_contact_mat.rows()) = ineq_contact_mat;
      ineq_vec.head(ineq_trq_mat.rows()) = ineq_trq_vec;
      ineq_vec.tail(ineq_contact_mat.rows()) = ineq_contact_vec;

    } else {
      // trq limit: o, contact: x
      ineq_mat = ineq_trq_mat;
      ineq_vec = ineq_trq_vec;
    }
  }

  // set up QP formulation & solve QP
  // quadprog QP formulation template Eq.
  /*
       min
          0.5 * x G x + g0 x
       s.t.
          CE^T x + ce0 = 0
          CI^T x + ci0 >= 0
      */
  num_qp_vars_ = cost_mat.cols();
  num_eq_const_ = eq_mat.rows();
  num_ineq_const_ = ineq_mat.rows();

  qddot_sol_.setZero(num_qdot_);
  rf_sol_.setZero(dim_contact_);

  x_.resize(num_qp_vars_);
  G_.resize(num_qp_vars_, num_qp_vars_);
  g0_.resize(num_qp_vars_);
  CE_.resize(num_qp_vars_, num_eq_const_);
  ce0_.resize(num_eq_const_);
  CI_.resize(num_qp_vars_, num_ineq_const_);
  ci0_.resize(num_ineq_const_);

  _SetQPCost(cost_mat, cost_vec);
  _SetQPEqualityConstraint(eq_mat, eq_vec);
  _SetQPInEqualityConstraint(ineq_mat, ineq_vec);
  _SolveQP();

  // compute torque command
  if (b_contact_) {
    // contact: o
    trq_cmd_ = sa_ni_trc_bar.transpose() * snf_ *
               (A_ * qddot_sol_ + ni.transpose() * (cori_ + grav_) -
                (jc_mat * ni).transpose() * rf_sol_ +
                ji_mat_transpose_lambda_int_jidot_qdot_vec);
  } else {
    // contact: x
    trq_cmd_ = sa_ni_trc_bar.transpose() * snf_ *
               (A_ * qddot_sol_ + ni.transpose() * (cori_ + grav_) +
                ji_mat_transpose_lambda_int_jidot_qdot_vec);
  }
}

void IHWBC::_SetQPCost(const Eigen::MatrixXd &cost_mat,
                       const Eigen::VectorXd &cost_vec) {
  for (int i(0); i < num_qp_vars_; ++i) {
    for (int j(0); j < num_qp_vars_; ++j) {
      G_[i][j] = cost_mat(i, j);
    }
    g0_[i] = cost_vec[i];
  }
}

void IHWBC::_SetQPEqualityConstraint(const Eigen::MatrixXd &eq_mat,
                                     const Eigen::VectorXd &eq_vec) {
  for (int i(0); i < num_eq_const_; ++i) {
    for (int j(0); j < num_qp_vars_; ++j) {
      CE_[j][i] = eq_mat(i, j);
    }
    ce0_[i] = eq_vec[i];
  }
}

void IHWBC::_SetQPInEqualityConstraint(const Eigen::MatrixXd &ineq_mat,
                                       const Eigen::VectorXd &ineq_vec) {
  for (int i(0); i < num_ineq_const_; ++i) {
    for (int j(0); j < num_qp_vars_; ++j) {
      CI_[j][i] = ineq_mat(i, j);
    }
    ci0_[i] = ineq_vec[i];
  }
}

void IHWBC::_SolveQP() {
  double qp_result = solve_quadprog(G_, g0_, CE_, ce0_, CI_, ci0_, x_);

  Eigen::VectorXd qp_sol = Eigen::VectorXd::Zero(num_qp_vars_);
  for (int i(0); i < num_qp_vars_; ++i)
    qp_sol[i] = x_[i];

  qddot_sol_ = qp_sol.head(num_qdot_);
  rf_sol_ = qp_sol.tail(dim_contact_);
}

void IHWBC::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node["qp"], "lambda_qddot", lambda_qddot_);
    util::ReadParameter(node["qp"], "lambda_rf", lambda_rf_);
    util::ReadParameter(node["qp"], "b_trq_limit", b_trq_limit_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}