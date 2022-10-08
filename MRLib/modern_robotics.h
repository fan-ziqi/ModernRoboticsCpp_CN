#pragma once

#include <Eigen/Dense>
#include <vector>

namespace mr
{
	/*--------------------第3章 刚体运动 P69--------------------*/

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
	 * 输入: 旋转的指数坐标
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
	 * 功能: 返回Frobenius范数以描述M到SO(3)的距离
	 *      注:特殊正交群SO(3)也称为旋转矩阵群，是所有3x3旋转矩阵R的集合
	 * 输入: 3x3矩阵M
	 * 输出: 使用以下方法计算M到SO(3)的距离:
	 *      如果 det(M) <= 0, 返回一个比较大的数.
	 *      如果 det(M) > 0, 返回norm(M^T*M - I).
	 */
	double DistanceToSO3(const Eigen::Matrix3d& M);

	/*
	 * 功能: 检查M是否接近或位于SO(3)上
	 *      注:特殊正交群SO(3)也称为旋转矩阵群，是所有3x3实数矩阵R的集合
	 * 输入: 3x3矩阵M
	 * 输出: 如果M接近或位于SO(3)上，返回true
	 */
	bool TestIfSO3(const Eigen::Matrix3d& M);

	/*
	 * 功能: 返回一个矩阵在SO(3)中的投影
	 *      注:特殊正交群SO(3)也称为旋转矩阵群，是所有3x3实数矩阵R的集合
	 * 输入: M:一个在SO(3)附近的矩阵，将要被投影到SO(3)
	 * 返回: SO(3)中最接近的矩阵R。该函数使用奇异值分解，仅适用于接近SO(3)的矩阵。
	 *      (详见 http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
	 */
	Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd& M);

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
	 * 返回: 齐次变换矩阵
	 */
	Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat);

	/*
	 * 功能: 计算齐次变换矩阵T∈SE(3)的矩阵对数semat3∈se(3)
	 * 输入: 齐次变换矩阵T
	 * 返回: T的矩阵对数semat3
	 */
	Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T);

	/*
	 * 功能: 返回Frobenius范数以描述T到SE(3)的距离
	 *      注:特殊欧式群SE(3)也称为刚体运动群或齐次变换矩阵群，是所有4x4实数矩阵T的集合
	 * 输入: 4x4矩阵T
	 * 输出: 使用以下方法计算T到SE(3)的距离:
	 *      matR为T的左上角3x3子矩阵
	 *      如果 det(matR) <= 0, 返回一个比较大的数.
	 *      如果 det(matR) > 0, 将左上角的3x3子矩阵替换为matR^T*matR
	 *                         并将第四列的前三行设置为0，然后返回norm(T - I)
	 */
	double DistanceToSE3(const Eigen::Matrix4d& T);

	/*
	 * 功能: 检查T是否接近或位于SE(3)上
	 *      注:特殊欧式群SE(3)也称为刚体运动群或齐次变换矩阵群，是所有4x4实数矩阵T的集合
	 * 输入: 4x4矩阵T
	 * 输出: 如果T接近或位于SE(3)上，返回true
	 */
	bool TestIfSE3(const Eigen::Matrix4d& T);

	/*
	 * 功能: 返回一个矩阵在SE(3)中的投影
	 *      注:特殊欧式群SE(3)也称为刚体运动群或齐次变换矩阵群，是所有4x4实数矩阵T的集合
	 * 输入: M:一个在SE(3)附近的矩阵，将要被投影到SE(3)
	 * 返回: SE(3)中最接近的矩阵T。该函数使用奇异值分解，仅适用于接近SE(3)的矩阵。
	 *      (详见 http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
	 */
	Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd& M);

	/*--------------------第4章 正向运动学 P99--------------------*/

	/*
	 * 功能: 给定末端的初始位形M,末端坐标系下的关节旋量Blist,以及关节值thetalist,计算末端坐标系
	 * 输入: 末端的初始位形M(位置和方向), 末端坐标系下的关节旋量Blist, 关节值thetalist
	 * 返回: 当关节位于指定值时，末端坐标系的变换矩阵
	 * 注:   FK表示正向运动学
	 */
	Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M,
	                         const Eigen::MatrixXd& Blist,
	                         const Eigen::VectorXd& thetaList);

	/*
	 * 功能: 给定末端的初始位形M,空间坐标系下的关节旋量Slist,以及关节值thetalist,计算末端坐标系
	 * 输入: 末端的初始位形M(位置和方向), 空间坐标系下的关节旋量Slist, 关节值thetalist
	 * 返回: 当关节位于指定值时，末端坐标系的变换矩阵
	 * 注:   FK表示正向运动学
	 */
	Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M,
							  const Eigen::MatrixXd& Slist,
							  const Eigen::VectorXd& thetaList);

	/*--------------------第5章 一阶运动学与静力学 P125--------------------*/

	/*
	 * 功能: 给定物体坐标系下描述的各关节旋量Bi及关节角, 计算物体雅可比Jb(θ)∈R(6xn)
	 * 输入: 物体坐标系下的各关节旋量Bi, 关节角
	 * 返回: 6xn 物体雅可比
	 */
	Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist,
	                             const Eigen::MatrixXd& thetaList);

	/*
	 * 功能: 给定空间坐标系下描述的各关节旋量Si及关节角, 计算空间雅可比Jb(θ)∈R(6xn)
	 * 输入: 空间坐标系下的各关节旋量Si, 关节角
	 * 返回: 6xn 空间雅可比
	 */
	Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist,
								  const Eigen::MatrixXd& thetaList);

	/*--------------------第6章 逆运动学 P144--------------------*/

	/*
	 * 功能: 已知末端坐标系中描述的关节旋量Bi、末端初始位形M、预期的末端位形T、
	 *      关节角的初始估计值θ0、以及最小误差εω和εv，利用迭代牛顿-拉夫森算法计算逆运动学。
	 * 输入:
	 *      Blist: 末端坐标系下的关节旋量(每个轴为一列)
	 *	    M: 末端初始位形
	 *	    T: 预期的末端位形
	 *	    thetalist[in]: 关节角的初始估计值
	 *	    emog: 末端坐标系中末端方向的小正误差。返回的关节角度必须使末端方向误差小于eomg
	 *	    ev:   末端坐标系中末端位置的小正误差。返回的关节角度必须使末端位置误差小于ev
	 * 输出:
	 *	    success: TRUE代表找到了解，FALSE代表运行了设定的最大迭代次数，但没有在公差eomg和ev内找到解。
	 *	    thetalist[out]: 在指定误差内的结果关节角
	 */
	bool IKinBody(const Eigen::MatrixXd& Blist,
	              const Eigen::MatrixXd& M,
	              const Eigen::MatrixXd& T,
	              Eigen::VectorXd& thetalist,
	              double eomg, double ev);

	/*
	 * 功能: 已知空间坐标系中描述的关节旋量Bi、末端初始位形M、预期的末端位形T、
	 *      关节角的初始估计值θ0、以及最小误差εω和εv，利用迭代牛顿-拉夫森算法计算逆运动学。
	 * 输入:
	 *      Blist: 空间坐标系下的关节旋量(每个轴为一列)
	 *	    M: 末端初始位形
	 *	    T: 预期的末端位形
	 *	    thetalist[in][out]: 关节角的初始估计值和在指定误差内的结果关节角
	 *	    emog: 空间坐标系中末端方向的小正误差。返回的关节角度必须使末端方向误差小于eomg
	 *	    ev:   空间坐标系中末端位置的小正误差。返回的关节角度必须使末端位置误差小于ev
	 * 输出:
	 *	    success: TRUE代表找到了解，FALSE代表运行了设定的最大迭代次数，但没有在公差eomg和ev内找到解。
	 */
	bool IKinSpace(const Eigen::MatrixXd& Slist,
	               const Eigen::MatrixXd& M,
	               const Eigen::MatrixXd& T,
	               Eigen::VectorXd& thetalist,
	               double eomg, double ev);

	/*--------------------第8章 开链动力学 P197--------------------*/

	/*
	 * 功能: 计算给定6矢量的6x6矩阵[adV]
	 * 输入: Eigen::VectorXd (6x1) (空间速度)
	 * 输出: Eigen::MatrixXd (6x6)
	 * 注: 可用于计算李括号 [V1, V2] = [adV1]V2
	 */
	Eigen::MatrixXd ad(Eigen::VectorXd V);

	/*
	 * 功能: 使用向前向后的牛顿-欧拉迭代逆动力学求解下述方程，计算所需关节力-力矩的n维向量τ:
	 *      taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) + g(thetalist) + Jtr(thetalist) * Ftip
	 * 输入:
	 *      thetalist: n维关节角度向量0
	 *      dthetalist: n维关节速度向量0'
	 *      ddthetalist: n维关节加速度向量0''
	 *      g: 重力向量G
	 *      Ftip: 在末端执行器坐标系{n+1}中表示的由末端执行器作用于环境的力旋量
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 * 输出:
	 *      taulist: 所需关节力-力矩的n维向量τ
	 */
	Eigen::VectorXd InverseDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist,
	                                const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist,
	                                const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist);

	/*
	 * 功能: 此函数调用n次InverseDynamics逐列构建质量矩阵M(θ)。
	 *      在第i次调用时，令列向量ddthelist中第i行为1，其他行全为0；且令g=0、Ftip=0、ddthetalist=0。
	 *      第i次返回的τ向量是M(θ)的第i列。
	 * 输入:
	 *      thetalist: n维关节角度向量0
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 * 输出:
	 *      M: 给定thetalist配置下的n关节串联链惯性矩阵M(θ)
	 */
	Eigen::MatrixXd MassMatrix(const Eigen::VectorXd& thetalist,
							   const std::vector<Eigen::MatrixXd>& Mlist,
							   const std::vector<Eigen::MatrixXd>& Glist,
							   const Eigen::MatrixXd& Slist);

	/*
	 * 功能: 调用InverseDynamics，且令g=0、Ftip=0、ddthetalist=0
	 * 输入:
	 *      thetalist: n维关节角度向量0
	 *      dthetalist: n维关节速度向量0'
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 * 输出:
	 *      c: 给定thetalist和dtetalist下的科氏向量和向心项c(0,0')
	 */
	Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd& thetalist,
									   const Eigen::VectorXd& dthetalist,
									   const std::vector<Eigen::MatrixXd>& Mlist,
									   const std::vector<Eigen::MatrixXd>& Glist,
									   const Eigen::MatrixXd& Slist);

	/*
	 * 功能: 调用InverseDynamics，且令Ftip=0、dthetalist=0、ddthetalist=0
	 * 输入:
	 *      thetalist: n维关节角度向量0
	 *      g: 重力向量G
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 * 输出:
	 *      grav: 重力项g(0)
	 */
	Eigen::VectorXd GravityForces(const Eigen::VectorXd& thetalist,
	                              const Eigen::VectorXd& g,
	                              const std::vector<Eigen::MatrixXd>& Mlist,
	                              const std::vector<Eigen::MatrixXd>& Glist,
	                              const Eigen::MatrixXd& Slist);

	/*
	 * 功能: 调用InverseDynamics，且令g=0、dthetalist=0、ddthetalist=0
	 * 输入:
	 *      thetalist: n维关节角度向量0
	 *      Ftip: 在末端执行器坐标系{n+1}中表示的由末端执行器作用于环境的力旋量
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 *
	 * 输出:
	 *      JTFtip: 仅创建末端执行器力Ftip所需的关节力和力矩
	 */
	Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd& thetalist,
									  const Eigen::VectorXd& Ftip,
									  const std::vector<Eigen::MatrixXd>& Mlist,
									  const std::vector<Eigen::MatrixXd>& Glist,
									  const Eigen::MatrixXd& Slist);

	/*
	 * 功能: 通过解下列方程来计算关节加速度ddthetalist:
	 *      Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist) - g(thetalist) - Jtr(thetalist) * Ftip
	 * 输入:
	 *      thetalist: n维关节角度向量0
	 *      dthetalist: n维关节速度向量0'
	 *      taulist: 所需关节力-力矩的n维向量τ
	 *      g: 重力向量G
	 *      Ftip: 在末端执行器坐标系{n+1}中表示的由末端执行器作用于环境的力旋量
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 * 输出:
	 *      ddthetalist: n维关节加速度向量0''
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
	 * 功能: 使用一阶欧拉积分计算下一时间步的关节角度和速度
	 * 输入:
	 *      thetalist[in]: n维关节角度向量0
	 *      dthetalist[in]: n维关节速度向量0'
	 *	    ddthetalist: n维关节加速度向量0''
	 *      dt: 时间步长δt
	 * 输出:
	 *      thetalist[out]: 一阶欧拉积分后的关节角度向量
	 *      dthetalist[out]: 一阶欧拉积分后的关节速度向量
	 */
	void EulerStep(Eigen::VectorXd& thetalist,
	               Eigen::VectorXd& dthetalist,
	               const Eigen::VectorXd& ddthetalist,
	               double dt);

	/*
	 * 功能: 使用逆动力学计算串联链生成指定轨迹所需要的关节力-力矩
	 * 输入:
	 *      thetamat: 关节角度变量0的一个Nxn矩阵(N:轨迹时间步点数; n:机器人关节数量)
	 *                其中第i行对应于时间t=(i-1)δt处关节变量的n维矩阵(δt为时间步长)
	 *      dthetamat: 关节速度向量0'的一个Nxn矩阵
	 *      ddthetamat: 关节加速度向量0''的一个Nxn矩阵
	 *	    g: 重力向量G
	 *	    Ftipmat: 末端执行器施加力旋量的Nx6矩阵(若没有末端力，需要输入一个零矩阵)
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 * 输出:
	 *      taumat: Nxn关节力-力矩矩阵τ(t)，其中N行中的每一行是每个时间步的关节力-力矩矢量。
	 *              该矩阵表示生成指定轨迹所需要的关节力-力矩τ(t)，其中轨迹由0(t)和Ftip(t)指定。
	 *              注意，没有必要指定δt。速度0'(t)和加速度0''(t)需要与0(t)一致
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
	 * 功能: 给定关节力-力矩，使用欧拉积分对机器人的数值方程进行数值积分，计算串联链的运动。
	 * 输入:
	 *      thetalist: n维初始关节角度向量0(0)
	 *      dthetalist: n维初始关节速度向量0'(0)
	 *      taumat: Nxn关节力-力矩矩阵τ(t)，其中N行中的每一行是每个时间步的关节力-力矩矢量。
	 *	    g: 重力向量G
	 *	    Ftipmat: 末端执行器施加力旋量的Nx6矩阵(若没有末端力，需要输入一个零矩阵)
	 *      Mlist: 变换矩阵M(i-1,i)组成的列表,其中M(i-1,i)指定了当机器人处于其原始位置时连杆质心坐标系{i}相对于{i-1}的位形
	 *      Glist: 连杆空间惯量矩阵Gi组成的列表
	 *      Slist: 在基座坐标系中表示的关节旋量轴Si组成的列表
	 *	    dt: 时间步长δt
	 *	    intRes: 积分分辨率是每个时间步长中积分的次数。必须是大于或等于1的整数值
	 *
	 * 输出:
	 *      std::vector 类型的 [thetamat, dthetamat]
	 *      thetamat: 由给定关节力-力矩生成的一个Nxn关节角度矩阵，第i行对应n维向量0((i-1)δt)
	 *      dthetamat: 关节速度向量0'的一个Nxn矩阵，第i行对应n维向量0'((i-1)δt)
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

	/*--------------------第9章 轨迹生成 P216--------------------*/

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

	/*--------------------第11章 机器人控制 P287--------------------*/

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

	/*--------------------其他--------------------*/

	/*
	 * 功能: 判断值是否可以忽略不计(接近于0)
	 * 输入: 被检查的值(double类型)
	 * 返回: true-可以忽略不计，false-不能忽略不计
	 */
	bool NearZero(const double val);

	/*
	 * 功能: 返回输入矩阵的标准化
	 * 输入: Eigen::MatrixXd
	 * 输出: Eigen::MatrixXd
	 * 注: 对于行向量，使用MatrixXd代替VectorXd
	 */
	Eigen::MatrixXd Normalize(Eigen::MatrixXd V);

	/*
	 * 功能: 获得旋转矩阵
	 * 输入: 角度(弧度制)
	 * 输出: 3x3旋转矩阵
	 */
	Eigen::Matrix3d rotx(const double t);
	Eigen::Matrix3d roty(const double t);
	Eigen::Matrix3d rotz(const double t);
}
