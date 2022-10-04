#pragma once

#include <Eigen/Dense>
#include <vector>

namespace mr {

/*
 * 功能: Find if the value is negligible enough to consider 0
 * 输入: value to be checked as a double
 * 返回: Boolean of true-ignore or false-can't ignore
 */
bool NearZero(const double val);

/*
 * 功能: Calculate the 6x6 matrix [adV] of the given 6-vector
 * 输入: Eigen::VectorXd (6x1)
 * 输出: Eigen::MatrixXd (6x6)
 * 注: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2
 */
Eigen::MatrixXd ad(Eigen::VectorXd V);


/*
 * 功能: 返回 a normalized version of the 输入 vector
 * 输入: Eigen::MatrixXd
 * 输出: Eigen::MatrixXd
 * 注: MatrixXd is used instead of VectorXd for the case of row vectors
 * 		Requires a copy
 *		Useful because of the MatrixXd casting
 */
Eigen::MatrixXd Normalize(Eigen::MatrixXd V);


/*
 * 功能: 计算旋转矩阵R的逆矩阵
 * 输入: 旋转矩阵R
 * 返回: R的逆矩阵
 */
Eigen::MatrixXd RotInv(const Eigen::MatrixXd& R);


/*
 * 功能: 将一个三维向量omg转换为3x3反对称矩阵
 * 输入: Eigen::Vector3d 3x1 三维向量(角速度矢量)
 * 返回: Eigen::MatrixXd 3x3 反对称矩阵
 */
Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg);


/*
 * 功能: 将3x3反对称矩阵转换为一个三维向量omg(角速度矢量)
 * 输入: Eigen::MatrixXd 3x3 反对称矩阵
 * 返回: Eigen::Vector3d 3x1 三维向量(角速度矢量)
 */
Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat);


/*
 * 功能: 从旋转的指数坐标expc3的三维向量ωθ中提取旋转轴线ω和旋转角度θ
 * 输入: 指数旋转(旋转轴和旋转角度的旋转矩阵)
 * 返回: 旋转轴和旋转角度 [x, y, z, theta]
 */
Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3);


/*
 * 功能: 计算矩阵指数so3mat∈so(3)的旋转矩阵R∈SO(3)
 * 输入: 旋转的矩阵指数so3mat
 * 返回: 旋转矩阵R
 */
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat);


/* 功能: 计算旋转矩阵R∈SO(3)的矩阵对数so3mat∈so(3)
 * 输入: 旋转矩阵R
 * 返回: 旋转的矩阵对数so3mat
 */
Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R);


/*
 * 功能: 将旋转矩阵R∈SO(3)和位置向量p∈R3组合成齐次变换矩阵
 * 输入: 旋转矩阵R, 位置向量p
 * 返回: 齐次变换矩阵 T = [ [R, p],
 *						 [0, 1] ]
 */
Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R,
						  const Eigen::Vector3d& p);


/*
 * 功能: 从齐次变换矩阵T中分离旋转矩阵R∈SO(3)和位置向量p∈R3
 * 输入: 齐次变换矩阵
 * 返回: std::vector类型的矩阵 [旋转矩阵R, 位置向量p]
 */
std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T);


/*
 * 功能: 计算齐次变换矩阵T的逆矩阵
 * 输入: 齐次变换矩阵T
 * 返回: T的逆矩阵
 */
Eigen::MatrixXd TransInv(const Eigen::MatrixXd& T);


/*
 * 功能: 将六维向量形式的运动旋量V转换为se(3)矩阵
 * 输入: 运动旋量 [角速度, 线速度]
 * 返回: V的se(3)矩阵表示
 */
Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V);


/* 功能: 将se(3)矩阵转换为六维向量形式的运动旋量V
 * 输入: se(3)矩阵
 * 返回: 运动旋量V [角速度, 线速度]
 */
Eigen::VectorXd se3ToVec(const Eigen::MatrixXd& se3mat);


/*
 * 功能: 计算齐次变换矩阵T的6x6伴随矩阵[AdT], 用于更改空间速度矢量的参考坐标系
 * 输入: 4x4齐次变换矩阵 SE(3)
 * 返回: 6x6伴随矩阵
 */
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T);


/*
 * 输入螺旋轴的参数,将其转化为标准形式
 * 输入:
     q: 螺旋轴上一点q
     s: 表示螺旋轴方向的单位向量
     h: 螺旋杆的节距
 * 返回: 标准化的螺旋轴S表达形式
 */
Eigen::VectorXd ScrewToAxis(Eigen::Vector3d q,
                            Eigen::Vector3d s,
                            double h);


/*
 * 功能: 从六维向量形式的指数坐标S0中提取标准化的螺旋轴S以及沿轴线移动的距离0
 * 输入:
	expc6: 六维向量形式的指数坐标S0
 * 返回: 标准化的螺旋轴S, 沿轴线移动的距离0 [S, theta]
 * 注: 如果返回 std::map<S, theta> 会不会更好?
 */
Eigen::VectorXd AxisAng6(const Eigen::VectorXd& expc6);


/*
 * 功能: 计算与矩阵指数se3mat∈se(3)对应的齐次变换矩阵T∈SE(3) (螺旋轴的旋转扩展)
 * 输入: 指数坐标的se(3)矩阵表示(变换矩阵)
 * 返回: 表示旋转的6x6矩阵
 */
Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat);


/*
 * 功能: 计算齐次变换矩阵T∈SE(3)的矩阵对数semat3∈se(3)
 * 输入: 齐次变换矩阵T
 * 返回: T的矩阵对数semat3
 */
Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T);


/*
 * 功能: 给定末端的初始位形M,空间坐标系下的关节旋量Slist,以及关节值thetalist,计算末端坐标系
 * 输入: 末端的初始位形M(位置和方向), 空间坐标系下的关节旋量Slist, 关节值thetalist
 * 返回: 当关节位于指定值时，末端坐标系的变换矩阵
 * 注:   FK表示正向运动学
 */
Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M,
						  const Eigen::MatrixXd& Slist,
						  const Eigen::VectorXd& thetaList);

/*
 * 功能: 给定末端的初始位形M,末端坐标系下的关节旋量Blist,以及关节值thetalist,计算末端坐标系
 * 输入: 末端的初始位形M(位置和方向), 末端坐标系下的关节旋量Slist, 关节值thetalist
 * 返回: 当关节位于指定值时，末端坐标系的变换矩阵
 * 注:   FK表示正向运动学
 */
Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M,
						 const Eigen::MatrixXd& Blist,
						 const Eigen::VectorXd& thetaList);


/*
 * 功能: 给定空间坐标系下描述的各关节旋量Si及关节角, 计算空间雅可比Jb(θ)∈R(6xn)
 * 输入: 空间坐标系下的各关节旋量Si, 关节角
 * 返回: 6xn 空间雅可比
 */
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist,
							  const Eigen::MatrixXd& thetaList);


/*
 * 功能: 给定物体坐标系下描述的各关节旋量Bi及关节角, 计算物体雅可比Jb(θ)∈R(6xn)
 * 输入: 物体坐标系下的各关节旋量Bi, 关节角
 * 返回: 6xn 物体雅可比
 */
Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist,
							 const Eigen::MatrixXd& thetaList);


/*
 * 功能: 返回 projection of one matrix into SO(3)
 * 输入:
 * M:		A matrix near SO(3) to project to SO(3)
 * 返回: The closest matrix R that is in SO(3)
 * Projects a matrix mat to the closest matrix in SO(3) using singular-value decomposition
 * (see http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
 * This 功能 is only appropriate for matrices close to SO(3).
 */
Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd& M);


/*
 * 功能: 返回 projection of one matrix into SE(3)
 * 输入:
 * M:		A 4x4 matrix near SE(3) to project to SE(3)
 * 返回: The closest matrix T that is in SE(3)
 * Projects a matrix mat to the closest matrix in SO(3) using singular-value decomposition
 * (see http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
 * This 功能 is only appropriate for matrices close to SE(3).
 */
Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd& M);


/*
 * 功能: 返回 the Frobenius norm to describe the distance of M from the SO(3) manifold
 * 输入:
 * M: A 3x3 matrix
 * 输出:
 *	 the distance from mat to the SO(3) manifold using the following
 * method:
 *  If det(M) <= 0, return a large number.
 *  If det(M) > 0, return norm(M^T*M - I).
 */
double DistanceToSO3(const Eigen::Matrix3d& M);


/*
 * 功能: 返回 the Frobenius norm to describe the distance of mat from the SE(3) manifold
 * 输入:
 * T: A 4x4 matrix
 * 输出:
 *	 the distance from T to the SE(3) manifold using the following
 * method:
 *  Compute the determinant of matR, the top 3x3 submatrix of T.
 *  If det(matR) <= 0, return a large number.
 *  If det(matR) > 0, replace the top 3x3 submatrix of mat with matR^T*matR,
 *  and set the first three entries of the fourth column of mat to zero. Then
 *  return norm(T - I).
 */
double DistanceToSE3(const Eigen::Matrix4d& T);


/*
 * 功能: 返回 true if M is close to or on the manifold SO(3)
 * 输入:
 * M: A 3x3 matrix
 * 输出:
 *	 true if M is very close to or in SO(3), false otherwise
 */
bool TestIfSO3(const Eigen::Matrix3d& M);


/*
 * 功能: 返回 true if T is close to or on the manifold SE(3)
 * 输入:
 * M: A 4x4 matrix
 * 输出:
 *	 true if T is very close to or in SE(3), false otherwise
 */
bool TestIfSE3(const Eigen::Matrix4d& T);


/*
 * 功能: Computes inverse kinematics in the body frame for an open chain robot
 * 输入:
 *	Blist: The joint screw axes in the end-effector frame when the
 *         manipulator is at the home position, in the format of a
 *         matrix with axes as the columns
 *	M: The home configuration of the end-effector
 *	T: The desired end-effector configuration Tsd
 *	thetalist[in][out]: An initial guess and result output of joint angles that are close to
 *         satisfying Tsd
 *	emog: A small positive tolerance on the end-effector orientation
 *        error. The returned joint angles must give an end-effector
 *        orientation error less than eomg
 *	ev: A small positive tolerance on the end-effector linear position
 *      error. The returned joint angles must give an end-effector
 *      position error less than ev
 * 输出:
 *	success: A logical value where TRUE means that the function found
 *           a solution and FALSE means that it ran through the set
 *           number of maximum iterations without finding a solution
 *           within the tolerances eomg and ev.
 *	thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
 */
bool IKinBody(const Eigen::MatrixXd& Blist,
			  const Eigen::MatrixXd& M,
			  const Eigen::MatrixXd& T,
			  Eigen::VectorXd& thetalist,
			  double eomg, double ev);


/*
 * 功能: Computes inverse kinematics in the space frame for an open chain robot
 * 输入:
 *	Slist: The joint screw axes in the space frame when the
 *         manipulator is at the home position, in the format of a
 *         matrix with axes as the columns
 *	M: The home configuration of the end-effector
 *	T: The desired end-effector configuration Tsd
 *	thetalist[in][out]: An initial guess and result output of joint angles that are close to
 *         satisfying Tsd
 *	emog: A small positive tolerance on the end-effector orientation
 *        error. The returned joint angles must give an end-effector
 *        orientation error less than eomg
 *	ev: A small positive tolerance on the end-effector linear position
 *      error. The returned joint angles must give an end-effector
 *      position error less than ev
 * 输出:
 *	success: A logical value where TRUE means that the function found
 *           a solution and FALSE means that it ran through the set
 *           number of maximum iterations without finding a solution
 *           within the tolerances eomg and ev.
 *	thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
 */
bool IKinSpace(const Eigen::MatrixXd& Slist,
			   const Eigen::MatrixXd& M,
			   const Eigen::MatrixXd& T,
			   Eigen::VectorXd& thetalist,
			   double eomg, double ev);

/*
 * 功能: This 功能 uses forward-backward Newton-Euler iterations to solve the
 * equation:
 * taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
 *           + g(thetalist) + Jtr(thetalist) * Ftip
 * 输入:
 *  thetalist: n-vector of joint variables
 *  dthetalist: n-vector of joint rates
 *  ddthetalist: n-vector of joint accelerations
 *  g: Gravity vector g
 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * 输出:
 *  taulist: The n-vector of required joint forces/torques
 *
 */
Eigen::VectorXd InverseDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist,
                                const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
                                const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist);

/*
 * 功能: This 功能 calls InverseDynamics with Ftip = 0, dthetalist = 0, and
 *   ddthetalist = 0. The purpose is to calculate one important term in the dynamics equation
 * 输入:
 *  thetalist: n-vector of joint variables
 *  g: Gravity vector g
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * 输出:
 *  grav: The 3-vector showing the effect force of gravity to the dynamics
 *
 */
Eigen::VectorXd GravityForces(const Eigen::VectorXd& thetalist,
							  const Eigen::VectorXd& g,
							  const std::vector<Eigen::MatrixXd>& Mlist,
							  const std::vector<Eigen::MatrixXd>& Glist,
							  const Eigen::MatrixXd& Slist);

/*
 * 功能: This 功能 calls InverseDynamics n times, each time passing a
 * ddthetalist vector with a single element equal to one and all other
 * 输入 set to zero. Each call of InverseDynamics generates a single
 * column, and these columns are assembled to create the inertia matrix.
 *
 * 输入:
 *  thetalist: n-vector of joint variables
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * 输出:
 *  M: The numerical inertia matrix M(thetalist) of an n-joint serial
 *     chain at the given configuration thetalist.
 */
Eigen::MatrixXd MassMatrix(const Eigen::VectorXd& thetalist,
						   const std::vector<Eigen::MatrixXd>& Mlist,
						   const std::vector<Eigen::MatrixXd>& Glist,
						   const Eigen::MatrixXd& Slist);

/*
 * 功能: This 功能 calls InverseDynamics with g = 0, Ftip = 0, and
 * ddthetalist = 0.
 *
 * 输入:
 *  thetalist: n-vector of joint variables
 *  dthetalist: A list of joint rates
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * 输出:
 *  c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
 *     terms for a given thetalist and dthetalist.
 */
Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd& thetalist,
								   const Eigen::VectorXd& dthetalist,
								   const std::vector<Eigen::MatrixXd>& Mlist,
								   const std::vector<Eigen::MatrixXd>& Glist,
								   const Eigen::MatrixXd& Slist);

/*
 * 功能: This 功能 calls InverseDynamics with g = 0, dthetalist = 0, and
 * ddthetalist = 0.
 *
 * 输入:
 *  thetalist: n-vector of joint variables
 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * 输出:
 *  JTFtip: The joint forces and torques required only to create the
 *     end-effector force Ftip.
 */
Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd& thetalist,
								  const Eigen::VectorXd& Ftip,
								  const std::vector<Eigen::MatrixXd>& Mlist,
								  const std::vector<Eigen::MatrixXd>& Glist,
								  const Eigen::MatrixXd& Slist);

/*
 * 功能: This 功能 computes ddthetalist by solving:
 * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist)
 *                                  - g(thetalist) - Jtr(thetalist) * Ftip
 * 输入:
 *  thetalist: n-vector of joint variables
 *  dthetalist: n-vector of joint rates
 *  taulist: An n-vector of joint forces/torques
 *  g: Gravity vector g
 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * 输出:
 *  ddthetalist: The resulting joint accelerations
 *
 */
Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd& thetalist,
                                const Eigen::VectorXd& dthetalist,
                                const Eigen::VectorXd& taulist,
                                const Eigen::VectorXd& g,
                                const Eigen::VectorXd& Ftip,
                                const std::vector<Eigen::MatrixXd>& Mlist,
                                const std::vector<Eigen::MatrixXd>& Glist,
                                const Eigen::MatrixXd& Slist);


/*
 * 功能: Compute the joint angles and velocities at the next timestep using
    first order Euler integration
 * 输入:
 *  thetalist[in]: n-vector of joint variables
 *  dthetalist[in]: n-vector of joint rates
 *	ddthetalist: n-vector of joint accelerations
 *  dt: The timestep delta t
 *
 * 输出:
 *  thetalist[out]: Vector of joint variables after dt from first order Euler integration
 *  dthetalist[out]: Vector of joint rates after dt from first order Euler integration
 */
void EulerStep(Eigen::VectorXd& thetalist,
               Eigen::VectorXd& dthetalist,
               const Eigen::VectorXd& ddthetalist,
               double dt);


/*
 * 功能: Compute the joint forces/torques required to move the serial chain along the given
 *	trajectory using inverse dynamics
 * 输入:
 *  thetamat: An N x n matrix of robot joint variables (N: no. of trajecoty time step points; n: no. of robot joints
 *  dthetamat: An N x n matrix of robot joint velocities
 *  ddthetamat: An N x n matrix of robot joint accelerations
 *	g: Gravity vector g
 *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
 *			 the user should 输入 a zero matrix)
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * 输出:
 *  taumat: The N x n matrix of joint forces/torques for the specified trajectory, where each of the N rows is the vector
 *			of joint forces/torques at each time step
 */
Eigen::MatrixXd InverseDynamicsTrajectory(const Eigen::MatrixXd& thetamat,
                                          const Eigen::MatrixXd& dthetamat,
                                          const Eigen::MatrixXd& ddthetamat,
                                          const Eigen::VectorXd& g,
                                          const Eigen::MatrixXd& Ftipmat,
                                          const std::vector<Eigen::MatrixXd>& Mlist,
                                          const std::vector<Eigen::MatrixXd>& Glist,
                                          const Eigen::MatrixXd& Slist);


/*
 * 功能: Compute the motion of a serial chain given an open-loop history of joint forces/torques
 * 输入:
 *  thetalist: n-vector of initial joint variables
 *  dthetalist: n-vector of initial joint rates
 *  taumat: An N x n matrix of joint forces/torques, where each row is is the joint effort at any time step
 *	g: Gravity vector g
 *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
 *			 the user should 输入 a zero matrix)
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *	dt: The timestep between consecutive joint forces/torques
 *	intRes: Integration resolution is the number of times integration (Euler) takes places between each time step.
 *			Must be an integer value greater than or equal to 1
 *
 * 输出: std::vector of [thetamat, dthetamat]
 *  thetamat: The N x n matrix of joint angles resulting from the specified joint forces/torques
 *  dthetamat: The N x n matrix of joint velocities
 */
std::vector<Eigen::MatrixXd> ForwardDynamicsTrajectory(const Eigen::VectorXd& thetalist,
                                                       const Eigen::VectorXd& dthetalist,
                                                       const Eigen::MatrixXd& taumat,
                                                       const Eigen::VectorXd& g,
                                                       const Eigen::MatrixXd& Ftipmat,
                                                       const std::vector<Eigen::MatrixXd>& Mlist,
                                                       const std::vector<Eigen::MatrixXd>& Glist,
                                                       const Eigen::MatrixXd& Slist,
                                                       double dt, int intRes);


/*
 * 功能: Compute the joint control torques at a particular time instant
 * 输入:
 *  thetalist: n-vector of joint variables
 *  dthetalist: n-vector of joint rates
 *	eint: n-vector of the time-integral of joint errors
 *	g: Gravity vector g
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *  thetalistd: n-vector of reference joint variables
 *  dthetalistd: n-vector of reference joint rates
 *  ddthetalistd: n-vector of reference joint accelerations
 *	Kp: The feedback proportional gain (identical for each joint)
 *	Ki: The feedback integral gain (identical for each joint)
 *	Kd: The feedback derivative gain (identical for each joint)
 *
 * 输出:
 *  tau_computed: The vector of joint forces/torques computed by the feedback
 *				  linearizing controller at the current instant
 */
Eigen::VectorXd ComputedTorque(const Eigen::VectorXd& thetalist,
                               const Eigen::VectorXd& dthetalist,
                               const Eigen::VectorXd& eint,
                               const Eigen::VectorXd& g,
                               const std::vector<Eigen::MatrixXd>& Mlist,
                               const std::vector<Eigen::MatrixXd>& Glist,
                               const Eigen::MatrixXd& Slist,
                               const Eigen::VectorXd& thetalistd,
                               const Eigen::VectorXd& dthetalistd,
                               const Eigen::VectorXd& ddthetalistd,
                               double Kp, double Ki, double Kd);


/*
 * 功能: Compute s(t) for a cubic time scaling
 * 输入:
 *  Tf: Total time of the motion in seconds from rest to rest
 *  t: The current time t satisfying 0 < t < Tf
 *
 * 输出:
 *  st: The path parameter corresponding to a third-order
 *      polynomial motion that begins and ends at zero velocity
 */
double CubicTimeScaling(double Tf, double t);


/*
 * 功能: Compute s(t) for a quintic time scaling
 * 输入:
 *  Tf: Total time of the motion in seconds from rest to rest
 *  t: The current time t satisfying 0 < t < Tf
 *
 * 输出:
 *  st: The path parameter corresponding to a fifth-order
 *      polynomial motion that begins and ends at zero velocity
 *	    and zero acceleration
 */
double QuinticTimeScaling(double Tf, double t);


/*
 * 功能: Compute a straight-line trajectory in joint space
 * 输入:
 *  thetastart: The initial joint variables
 *  thetaend: The final joint variables
 *  Tf: Total time of the motion in seconds from rest to rest
 *	N: The number of points N > 1 (Start and stop) in the discrete
 *     representation of the trajectory
 *  method: The time-scaling method, where 3 indicates cubic (third-
 *          order polynomial) time scaling and 5 indicates quintic
 *          (fifth-order polynomial) time scaling
 *
 * 输出:
 *  traj: A trajectory as an N x n matrix, where each row is an n-vector
 *        of joint variables at an instant in time. The first row is
 *        thetastart and the Nth row is thetaend . The elapsed time
 *        between each row is Tf / (N - 1)
 */
Eigen::MatrixXd JointTrajectory(const Eigen::VectorXd& thetastart,
                                const Eigen::VectorXd& thetaend,
                                double Tf, int N, int method);


/*
 * 功能: Compute a trajectory as a list of N SE(3) matrices corresponding to
 *			 the screw motion about a space screw axis
 * 输入:
 *  Xstart: The initial end-effector configuration
 *  Xend: The final end-effector configuration
 *  Tf: Total time of the motion in seconds from rest to rest
 *	N: The number of points N > 1 (Start and stop) in the discrete
 *     representation of the trajectory
 *  method: The time-scaling method, where 3 indicates cubic (third-
 *          order polynomial) time scaling and 5 indicates quintic
 *          (fifth-order polynomial) time scaling
 *
 * 输出:
 *  traj: The discretized trajectory as a list of N matrices in SE(3)
 *        separated in time by Tf/(N-1). The first in the list is Xstart
 *        and the Nth is Xend
 */
std::vector<Eigen::MatrixXd> ScrewTrajectory(const Eigen::MatrixXd& Xstart,
                                             const Eigen::MatrixXd& Xend,
                                             double Tf, int N, int method);


/*
 * 功能: Compute a trajectory as a list of N SE(3) matrices corresponding to
 *			 the origin of the end-effector frame following a straight line
 * 输入:
 *  Xstart: The initial end-effector configuration
 *  Xend: The final end-effector configuration
 *  Tf: Total time of the motion in seconds from rest to rest
 *	N: The number of points N > 1 (Start and stop) in the discrete
 *     representation of the trajectory
 *  method: The time-scaling method, where 3 indicates cubic (third-
 *          order polynomial) time scaling and 5 indicates quintic
 *          (fifth-order polynomial) time scaling
 *
 * 输出:
 *  traj: The discretized trajectory as a list of N matrices in SE(3)
 *        separated in time by Tf/(N-1). The first in the list is Xstart
 *        and the Nth is Xend
 * 注s:
 *	This 功能 is similar to ScrewTrajectory, except the origin of the
 *  end-effector frame follows a straight line, decoupled from the rotational
 *  motion.
 */
std::vector<Eigen::MatrixXd> CartesianTrajectory(const Eigen::MatrixXd& Xstart,
                                                 const Eigen::MatrixXd& Xend,
                                                 double Tf, int N, int method);


/*
 * 功能: Compute the motion of a serial chain given an open-loop history of joint forces/torques
 * 输入:
 *  thetalist: n-vector of initial joint variables
 *  dthetalist: n-vector of initial joint rates
 *	g: Gravity vector g
 *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
 *			 the user should 输入 a zero matrix)
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *  thetamatd: An Nxn matrix of desired joint variables from the reference trajectory
 *  dthetamatd: An Nxn matrix of desired joint velocities
 *  ddthetamatd: An Nxn matrix of desired joint accelerations
 *	gtilde: The gravity vector based on the model of the actual robot (actual values given above)
 *  Mtildelist: The link frame locations based on the model of the actual robot (actual values given above)
 *  Gtildelist: The link spatial inertias based on the model of the actual robot (actual values given above)
 *	Kp: The feedback proportional gain (identical for each joint)
 *	Ki: The feedback integral gain (identical for each joint)
 *	Kd: The feedback derivative gain (identical for each joint)
 *	dt: The timestep between points on the reference trajectory
 *	intRes: Integration resolution is the number of times integration (Euler) takes places between each time step.
 *			Must be an integer value greater than or equal to 1
 *
 * 输出: std::vector of [taumat, thetamat]
 *  taumat: An Nxn matrix of the controllers commanded joint forces/ torques, where each row of n forces/torques
 *			  corresponds to a single time instant
 *  thetamat: The N x n matrix of actual joint angles
 */
std::vector<Eigen::MatrixXd> SimulateControl(const Eigen::VectorXd& thetalist,
                                             const Eigen::VectorXd& dthetalist,
                                             const Eigen::VectorXd& g,
                                             const Eigen::MatrixXd& Ftipmat,
                                             const std::vector<Eigen::MatrixXd>& Mlist,
                                             const std::vector<Eigen::MatrixXd>& Glist,
                                             const Eigen::MatrixXd& Slist,
                                             const Eigen::MatrixXd& thetamatd,
                                             const Eigen::MatrixXd& dthetamatd,
                                             const Eigen::MatrixXd& ddthetamatd,
                                             const Eigen::VectorXd& gtilde,
                                             const std::vector<Eigen::MatrixXd>& Mtildelist,
                                             const std::vector<Eigen::MatrixXd>& Gtildelist,
                                             double Kp, double Ki, double Kd, double dt, int intRes);

}
