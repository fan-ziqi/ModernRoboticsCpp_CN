#include "MRTest.h"

#include <iostream>
#include "Eigen/Dense"
#include "modern_robotics.h"

using namespace mr;
using namespace std;

/*--------------------第3章 刚体运动 P69--------------------*/

void RotInvTest()
{
	//旋转矩阵R
	Eigen::MatrixXd R(3, 3);
	R << 0, 0, 1,
			1, 0, 0,
			0, 1, 0;

	//计算其逆矩阵
	Eigen::MatrixXd invR = mr::RotInv(R);

	//正确结果
	Eigen::MatrixXd result(3, 3);
	result << 0, 1, 0,
			0, 0, 1,
			1, 0, 0;

	cout << "RotInvTest" << endl
	     << "The result is:" << endl << invR << endl
	     << "It should be:" << endl << result << endl;
}

void VecToso3Test()
{
	//三维向量(角速度矢量)
	Eigen::Vector3d omg(1, 2, 3);

	//生成反对称矩阵
	Eigen::Matrix3d so3mat = VecToso3(omg);

	//正确结果
	Eigen::Matrix3d result(3, 3);
	result << 0, -3, 2, 3, 0, -1, -2, 1, 0;

	cout << "VecToso3Test" << endl
	     << "The result is:" << endl << so3mat << endl
	     << "It should be:" << endl << result << endl;
}

void DistanceToSO3Test()
{
	Eigen::Matrix3d mat;
	mat << 1.0, 0.0, 0.0,
			0.0, 0.1, -0.95,
			0.0, 1.0, 0.1;

	//正确结果
	double result = 0.088353;

	double d = DistanceToSO3(mat);
	cout << "DistanceToSO3Test" << endl
	     << "The result is:" << endl << d << endl
	     << "It should be:" << endl << result << endl;
}

void TestIfSO3Test()
{
	Eigen::Matrix3d input;

	//正确结果
	bool result = false;
	input << 1.0, 0.0, 0.0,
			0.0, 0.1, -0.95,
			0.0, 1.0, 0.1;
	bool tmp_result = TestIfSO3(input);
	cout << "TestIfSO3Test" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

void TransInvTest()
{
	//齐次变换矩阵T
	Eigen::MatrixXd T(4, 4);
	T << 1, 0, 0, 0,
			0, 0, -1, 0,
			0, 1, 0, 3,
			0, 0, 0, 1;

	//计算其逆矩阵
	Eigen::MatrixXd invT = mr::TransInv(T);

	//正确结果
	Eigen::MatrixXd result(4, 4);
	result << 1, 0, 0, 0,
			0, 0, 1, -3,
			0, -1, 0, 0,
			0, 0, 0, 1;

	cout << "TransInvTest" << endl
	     << "The result is:" << endl << invT << endl
	     << "It should be:" << endl << result << endl;
}

void AdjointTest()
{
	Eigen::Matrix4d T;
	T << 1, 0, 0, 0,
			0, 0, -1, 0,
			0, 1, 0, 3,
			0, 0, 0, 1;

	//正确结果
	Eigen::MatrixXd result(6, 6);
	result <<
	       1, 0, 0, 0, 0, 0,
			0, 0, -1, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 3, 1, 0, 0,
			3, 0, 0, 0, 0, -1,
			0, 0, 0, 0, 1, 0;

	Eigen::MatrixXd tmp_result = Adjoint(T);

	cout << "AdjointTest" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

void ScrewToAxisTest()
{
	//螺旋轴上一点q
	Eigen::Vector3d q;
	q << 3, 0, 1;

	//表示螺旋轴方向的单位向量
	Eigen::Vector3d s;
	s << 0, 0, 1;

	//螺旋杆的节距
	double h = 2;

	//计算标准化的螺旋轴S表达形式
	Eigen::VectorXd S = mr::ScrewToAxis(q, s, h);

	//正确结果
	Eigen::VectorXd result(6);
	result << 0, 0, 1, 0, -3, 2;

	cout << "ScrewToAxisTest" << endl
	     << "The result is:" << endl << S << endl
	     << "It should be:" << endl << result << endl;
}

void AxisAng6Test()
{
	//六维向量形式的指数坐标S0
	Eigen::VectorXd input(6);
	input << 1.0, 0.0, 0.0, 1.0, 2.0, 3.0;

	//计算标准化的螺旋轴S, 沿轴线移动的距离0 [S, theta]
	Eigen::VectorXd output = mr::AxisAng6(input);

	//正确结果
	Eigen::VectorXd result(7);
	result << 1.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0;

	cout << "AxisAng6Test" << endl
	     << "The result is:" << endl << output << endl
	     << "It should be:" << endl << result << endl;
}

void MatrixLog6Test()
{
	//齐次变换矩阵T
	Eigen::MatrixXd T(4, 4);
	T << 1, 0, 0, 0,
			0, 0, -1, 0,
			0, 1, 0, 3,
			0, 0, 0, 1;

	//T的矩阵对数semat3
	Eigen::MatrixXd expmat = mr::MatrixLog6(T);

	//正确结果
	Eigen::MatrixXd result(4, 4);
	result << 0, 0, 0, 0,
			0, 0, -1.57079633, 2.35619449,
			0, 1.57079633, 0, 2.35619449,
			0, 0, 0, 0;

	cout << "MatrixLog6Test" << endl
	     << "The result is:" << endl << expmat << endl
	     << "It should be:" << endl << result << endl;
}

void DistanceToSE3Test()
{
	Eigen::Matrix4d input;

	//正确结果
	double result = 0.134931;
	input << 1.0, 0.0, 0.0, 1.2,
			0.0, 0.1, -0.95, 1.5,
			0.0, 1.0, 0.1, -0.9,
			0.0, 0.0, 0.1, 0.98;
	double tmp_result = DistanceToSE3(input);
	cout << "DistanceToSE3Test" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

void TestIfSE3Test()
{
	Eigen::Matrix4d input;

	//正确结果
	bool result = false;
	input << 1.0, 0.0, 0.0, 1.2,
			0.0, 0.1, -0.95, 1.5,
			0.0, 1.0, 0.1, -0.9,
			0.0, 0.0, 0.1, 0.98;
	bool tmp_result = TestIfSE3(input);
	cout << "TestIfSE3Test" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

/*--------------------第4章 正向运动学 P99--------------------*/

void FKinBodyTest()
{
	//末端的初始位形M
	Eigen::MatrixXd M(4, 4);
	M << -1, 0, 0, 0,
			0, 1, 0, 6,
			0, 0, -1, 2,
			0, 0, 0, 1;

	//末端坐标系下的关节旋量Blist
	Eigen::MatrixXd Blist(6, 3);
	Blist << 0, 0, 0,
			0, 0, 0,
			-1, 0, 1,
			2, 0, 0,
			0, 1, 0,
			0, 0, 0.1;

	//关节值thetalist
	Eigen::VectorXd thetaList(3);
	thetaList << M_PI / 2.0, 3, M_PI;

	//正向运动学
	Eigen::MatrixXd FKCal = mr::FKinBody(M, Blist, thetaList);

	//正确结果
	Eigen::MatrixXd result(4, 4);
	result << 0, 1, 0, -5,
			1, 0, 0, 4,
			0, 0, -1, 1.68584073,
			0, 0, 0, 1;

	cout << "FKinBodyTest" << endl
	     << "The result is:" << endl << FKCal << endl
	     << "It should be:" << endl << result << endl;
}

void FKinSpaceTest()
{
	//末端的初始位形M
	Eigen::MatrixXd M(4, 4);
	M << -1, 0, 0, 0,
			0, 1, 0, 6,
			0, 0, -1, 2,
			0, 0, 0, 1;

	//空间坐标系下的关节旋量Slist
	Eigen::MatrixXd Slist(6, 3);
	Slist << 0, 0, 0,
			0, 0, 0,
			1, 0, -1,
			4, 0, -6,
			0, 1, 0,
			0, 0, -0.1;

	//关节值thetalist
	Eigen::VectorXd thetaList(3);
	thetaList << M_PI / 2.0, 3, M_PI;

	//正向运动学
	Eigen::MatrixXd FKCal = mr::FKinSpace(M, Slist, thetaList);

	//正确结果
	Eigen::MatrixXd result(4, 4);
	result << 0, 1, 0, -5,
			1, 0, 0, 4,
			0, 0, -1, 1.68584073,
			0, 0, 0, 1;

	cout << "FKinSpaceTest" << endl
	     << "The result is:" << endl << FKCal << endl
	     << "It should be:" << endl << result << endl;
}

/*--------------------第5章 一阶运动学与静力学 P125--------------------*/

void JacobianBodyTest()
{
	//末端坐标系下的关节旋量Blist
	Eigen::MatrixXd Blist(6, 3);
	Blist << 0, 0, 0,
			0, 1, -1,
			1, 0, 0,
			0.0425, 0, 0,
			0.5515, 0, 0,
			0, -0.5515, 0.2720;

	//关节值thetalist
	Eigen::VectorXd thetalist(3);
	thetalist << 0, 0, 1.5708;

	//正确结果
	Eigen::MatrixXd result(6, 3);
	result << 1, 0, 0,
			0, 1, -1,
			0, 0, 0,
			0, -0.2795, 0,
			0.2795, 0, 0,
			-0.0425, -0.2720, 0.2720;

	//物体雅可比
	Eigen::MatrixXd Jb = mr::JacobianBody(Blist, thetalist);

	cout << "JacobianBodyTest" << endl
	     << "The result is:" << endl << Jb << endl
	     << "It should be:" << endl << result << endl;
}

void JacobianSpaceTest()
{
	//空间坐标系下的关节旋量Slist
	Eigen::MatrixXd Slist(6, 3);
	Slist << 0, 0, 0,
			0, 1, -1,
			1, 0, 0,
			0, -0.0711, 0.0711,
			0, 0, 0,
			0, 0, -0.2795;

	//关节值thetalist
	Eigen::VectorXd thetalist(3);
	thetalist << 1.0472, 1.0472, 1.0472;

	//正确结果
	Eigen::MatrixXd result(6, 3);
	result << 0, -0.866, 0.866,
			0, 0.5, -0.5,
			1, 0, 0,
			0, -0.0355, -0.0855,
			0, -0.0615, -0.1481,
			0, 0, -0.1398;

	//空间雅可比
	Eigen::MatrixXd Js = mr::JacobianSpace(Slist, thetalist);

	cout << "JacobianSpaceTest" << endl
	     << "The result is:" << endl << Js << endl
	     << "It should be:" << endl << result << endl;
}

/*--------------------第6章 逆运动学 P144--------------------*/

void IKinBodyTest()
{
	Eigen::MatrixXd BlistT(3, 6);
	BlistT << 0, 0, -1, 2, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 1, 0, 0, 0.1;
	Eigen::MatrixXd Blist = BlistT.transpose();
	Eigen::Matrix4d M;
	M << -1, 0, 0, 0,
			0, 1, 0, 6,
			0, 0, -1, 2,
			0, 0, 0, 1;
	Eigen::Matrix4d T;
	T << 0, 1, 0, -5,
			1, 0, 0, 4,
			0, 0, -1, 1.6858,
			0, 0, 0, 1;
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;
	double eomg = 0.01;
	double ev = 0.001;

	//正确结果
	bool b_result = true;
	Eigen::VectorXd theta_result(3);
	theta_result << 1.57073819, 2.999667, 3.14153913;

	bool iRet = mr::IKinBody(Blist, M, T, thetalist, eomg, ev);

	cout << "IKinBodyTest" << endl
	     << "The result is:" << endl << iRet << endl
	     << "It should be:" << endl << b_result << endl
	     << "The result is:" << endl << thetalist << endl
	     << "It should be:" << endl << theta_result << endl;
}

void IKinSpaceTest()
{
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 0, 0, 1, 4, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, -1, -6, 0, -0.1;
	Eigen::MatrixXd Slist = SlistT.transpose();
	Eigen::Matrix4d M;
	M << -1, 0, 0, 0,
			0, 1, 0, 6,
			0, 0, -1, 2,
			0, 0, 0, 1;
	Eigen::Matrix4d T;
	T << 0, 1, 0, -5,
			1, 0, 0, 4,
			0, 0, -1, 1.6858,
			0, 0, 0, 1;
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;
	double eomg = 0.01;
	double ev = 0.001;

	//正确结果
	bool b_result = true;
	Eigen::VectorXd theta_result(3);
	theta_result << 1.57073783, 2.99966384, 3.1415342;

	bool iRet = mr::IKinSpace(Slist, M, T, thetalist, eomg, ev);

	cout << "IKinSpaceTest" << endl
	     << "The result is:" << endl << iRet << endl
	     << "It should be:" << endl << b_result << endl
	     << "The result is:" << endl << thetalist << endl
	     << "It should be:" << endl << theta_result << endl;
}

/*--------------------第8章 开链动力学 P197--------------------*/

void adTest()
{
	Eigen::VectorXd V(6);
	V << 1, 2, 3, 4, 5, 6;

	//正确结果
	Eigen::MatrixXd result(6, 6);
	result << 0, -3, 2, 0, 0, 0,
			3, 0, -1, 0, 0, 0,
			-2, 1, 0, 0, 0, 0,
			0, -6, 5, 0, -3, 2,
			6, 0, -4, 3, 0, -1,
			-5, 4, 0, -2, 1, 0;

	Eigen::MatrixXd adV = ad(V);

	cout << "adTest" << endl
	     << "The result is:" << endl << adV << endl
	     << "It should be:" << endl << result << endl;
}

void InverseDynamicsTest()
{
	//3连杆机械臂算例
	//角度、速度、加速度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	Eigen::VectorXd ddthetalist(3);
	ddthetalist << 2, 1.5, 1;

	//g为在基座坐标系中的重力向量
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//在末端执行器坐标系{n+1}中表示的由末端执行器作用于环境的力旋量
	Eigen::VectorXd Ftip(6);
	Ftip << 1, 1, 1, 1, 1, 1;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	Eigen::VectorXd taulist = mr::InverseDynamics(thetalist, dthetalist, ddthetalist, g,
	                                              Ftip, Mlist, Glist, Slist);

	//正确结果
	Eigen::VectorXd result(3);
	result << 74.6962, -33.0677, -3.23057;

	cout << "InverseDynamicsTest" << endl
	     << "The result is:" << endl << taulist << endl
	     << "It should be:" << endl << result << endl;
}

void MassMatrixTest()
{
	//3连杆机械臂算例
	//角度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	Eigen::MatrixXd M = mr::MassMatrix(thetalist, Mlist, Glist, Slist);

	//正确结果
	Eigen::MatrixXd result(3, 3);
	result << 22.5433, -0.3071, -0.0072,
			-0.3071, 1.9685, 0.4322,
			-0.0072, 0.4322, 0.1916;

	cout << "MassMatrixTest" << endl
	     << "The result is:" << endl << M << endl
	     << "It should be:" << endl << result << endl;
}

void VelQuadraticForcesTest()
{
	//3连杆机械臂算例
	//角度、速度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	Eigen::VectorXd c = mr::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist);

	//正确结果
	Eigen::VectorXd result(3);
	result << 0.2645, -0.0551, -0.0069;

	cout << "VelQuadraticForcesTest" << endl
	     << "The result is:" << endl << c << endl
	     << "It should be:" << endl << result << endl;
}

void GravityForcesTest()
{
	//3连杆机械臂算例
	//角度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;

	//g为在基座坐标系中的重力向量
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	Eigen::VectorXd grav = mr::GravityForces(thetalist, g, Mlist, Glist, Slist);

	//正确结果
	Eigen::VectorXd result(3);
	result << 28.4033, -37.6409, -5.4416;

	cout << "GravityForcesTest" << endl
	     << "The result is:" << endl << grav << endl
	     << "It should be:" << endl << result << endl;
}

void EndEffectorForcesTest()
{
	//3连杆机械臂算例
	//角度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;

	//在末端执行器坐标系{n+1}中表示的由末端执行器作用于环境的力旋量
	Eigen::VectorXd Ftip(6);
	Ftip << 1, 1, 1, 1, 1, 1;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	Eigen::VectorXd JTFtip = mr::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

	//正确结果
	Eigen::VectorXd result(3);
	result << 1.4095, 1.8577, 1.3924;

	cout << "EndEffectorForcesTest" << endl
	     << "The result is:" << endl << JTFtip << endl
	     << "It should be:" << endl << result << endl;
}

void ForwardDynamicsTest()
{
	//3连杆机械臂算例
	//角度、速度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;

	//所需关节力-力矩的n维向量τ
	Eigen::VectorXd taulist(3);
	taulist << 0.5, 0.6, 0.7;

	//g为在基座坐标系中的重力向量
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//在末端执行器坐标系{n+1}中表示的由末端执行器作用于环境的力旋量
	Eigen::VectorXd Ftip(6);
	Ftip << 1, 1, 1, 1, 1, 1;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	Eigen::VectorXd ddthetalist = mr::ForwardDynamics(thetalist, dthetalist, taulist, g,
	                                                  Ftip, Mlist, Glist, Slist);

	//正确结果
	Eigen::VectorXd result(3);
	result << -0.9739, 25.5847, -32.9150;

	cout << "ForwardDynamicsTest" << endl
	     << "The result is:" << endl << ddthetalist << endl
	     << "It should be:" << endl << result << endl;
}

void EulerStepTest()
{
	//3连杆机械臂算例
	//角度、速度、加速度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	Eigen::VectorXd ddthetalist(3);
	ddthetalist << 2, 1.5, 1;
	double dt = 0.1;

	mr::EulerStep(thetalist, dthetalist, ddthetalist, dt);

	//正确结果
	Eigen::VectorXd result_thetalistNext(3);
	result_thetalistNext << 0.11, 0.12, 0.13;
	Eigen::VectorXd result_dthetalistNext(3);
	result_dthetalistNext << 0.3, 0.35, 0.4;

	cout << "EulerStepTest" << endl
	     << "The result is:" << endl << thetalist << endl
	     << "It should be:" << endl << result_thetalistNext << endl
	     << "The result is:" << endl << dthetalist << endl
	     << "It should be:" << endl << result_dthetalistNext << endl;
}

void InverseDynamicsTrajectoryTest()
{
	//3连杆机械臂算例
	int dof = 3;
	Eigen::VectorXd thetastart(dof);
	thetastart << 0, 0, 0;
	Eigen::VectorXd thetaend(dof);
	thetaend << M_PI / 2, M_PI / 2, M_PI / 2;
	double Tf = 3.0;
	int N = 1000;
	int method = 5;

	Eigen::MatrixXd traj = mr::JointTrajectory(thetastart, thetaend, Tf, N, method);
	Eigen::MatrixXd thetamat = traj;
	Eigen::MatrixXd dthetamat = Eigen::MatrixXd::Zero(N, dof);
	Eigen::MatrixXd ddthetamat = Eigen::MatrixXd::Zero(N, dof);
	double dt = Tf / (N - 1.0);
	for (int i = 0; i < N - 1; ++i)
	{
		dthetamat.row(i + 1) = (thetamat.row(i + 1) - thetamat.row(i)) / dt;
		ddthetamat.row(i + 1) = (dthetamat.row(i + 1) - dthetamat.row(i)) / dt;
	}

	//g为在基座坐标系中的重力向量
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;
	Eigen::MatrixXd Ftipmat = Eigen::MatrixXd::Zero(N, 6);

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	int numTest = 3;

	//正确结果
	Eigen::MatrixXd result(numTest, dof);
	Eigen::VectorXd tau_timestep_beg(3);
	tau_timestep_beg << 13.22970794, -36.262108, -4.181341;
	Eigen::VectorXd tau_timestep_mid(3);
	tau_timestep_mid << 115.55863434, -22.05129215, 1.00916115;
	Eigen::VectorXd tau_timestep_end(3);
	tau_timestep_end << 81.12700926, -23.20753925, 2.48432708;
	result << tau_timestep_beg.transpose(),
			tau_timestep_mid.transpose(),
			tau_timestep_end.transpose();

	Eigen::MatrixXd taumat = mr::InverseDynamicsTrajectory(thetamat, dthetamat, ddthetamat, g, Ftipmat, Mlist, Glist, Slist);
	Eigen::MatrixXd taumat_timestep(numTest, dof);
	taumat_timestep << taumat.row(0),
			taumat.row(int(N / 2) - 1),
			taumat.row(N - 1);
	cout << "InverseDynamicsTrajectoryTest" << endl
	     << "The result is:" << endl << taumat_timestep << endl
	     << "It should be:" << endl << result << endl;
}

void ForwardDynamicsTrajectoryTest()
{
	//3连杆机械臂算例
	//角度、速度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	int N = 10, dof = 3;
	Eigen::MatrixXd taumat(N, 3);
	taumat << 3.63, -6.58, -5.57,
			3.74, -5.55, -5.5,
			4.31, -0.68, -5.19,
			5.18, 5.63, -4.31,
			5.85, 8.17, -2.59,
			5.78, 2.79, -1.7,
			4.99, -5.3, -1.19,
			4.08, -9.41, 0.07,
			3.56, -10.1, 0.97,
			3.49, -9.41, 1.23;

	//g为在基座坐标系中的重力向量
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;
	Eigen::MatrixXd Ftipmat = Eigen::MatrixXd::Zero(N, 6);

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();
	double dt = 0.1;
	int intRes = 8;

	//正确结果
	Eigen::MatrixXd result_thetamat(N, dof);
	Eigen::MatrixXd result_dthetamat(N, dof);
	result_thetamat << 0.1, 0.1, 0.1,
			0.10643138, 0.2625997, -0.22664947,
			0.10197954, 0.71581297, -1.22521632,
			0.0801044, 1.33930884, -2.28074132,
			0.0282165, 2.11957376, -3.07544297,
			-0.07123855, 2.87726666, -3.83289684,
			-0.20136466, 3.397858, -4.83821609,
			-0.32380092, 3.73338535, -5.98695747,
			-0.41523262, 3.85883317, -7.01130559,
			-0.4638099, 3.63178793, -7.63190052;
	result_dthetamat << 0.1, 0.2, 0.3,
			0.01212502, 3.42975773, -7.74792602,
			-0.13052771, 5.55997471, -11.22722784,
			-0.35521041, 7.11775879, -9.18173035,
			-0.77358795, 8.17307573, -7.05744594,
			-1.2350231, 6.35907497, -8.99784746,
			-1.31426299, 4.07685875, -11.18480509,
			-1.06794821, 2.49227786, -11.69748583,
			-0.70264871, -0.55925705, -8.16067131,
			-0.1455669, -4.57149985, -3.43135114;

	std::vector<Eigen::MatrixXd> traj = mr::ForwardDynamicsTrajectory(thetalist, dthetalist, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes);
	Eigen::MatrixXd traj_theta = traj.at(0);
	Eigen::MatrixXd traj_dtheta = traj.at(1);

	cout << "ForwardDynamicsTrajectoryTest" << endl
	     << "The result is:" << endl << traj_theta << endl
	     << "It should be:" << endl << result_thetamat << endl
	     << "The result is:" << endl << traj_dtheta << endl
	     << "It should be:" << endl << result_dthetamat << endl;
}

/*--------------------第9章 轨迹生成 P216--------------------*/

void CubicTimeScalingTest()
{
	double Tf = 2.0;
	double t = 0.6;

	//正确结果
	double result = 0.216;

	double tmp_result = CubicTimeScaling(Tf, t);

	cout << "CubicTimeScalingTest" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

void QuinticTimeScalingTest()
{
	double Tf = 2.0;
	double t = 0.6;

	//正确结果
	double result = 0.16308;

	double tmp_result = QuinticTimeScaling(Tf, t);

	cout << "QuinticTimeScalingTest" << endl
	     << "The result is:" << endl << tmp_result << endl
	     << "It should be:" << endl << result << endl;
}

void JointTrajectoryTest()
{
	int dof = 8;
	Eigen::VectorXd thetastart(dof);
	thetastart << 1, 0, 0, 1, 1, 0.2, 0, 1;
	Eigen::VectorXd thetaend(dof);
	thetaend << 1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1;
	double Tf = 4.0;
	int N = 6;
	int method = 3;

	//正确结果
	Eigen::MatrixXd result(N, dof);
	result << 1, 0, 0, 1, 1, 0.2, 0, 1,
			1.0208, 0.052, 0.0624, 1.0104, 1.104, 0.3872, 0.0936, 1,
			1.0704, 0.176, 0.2112, 1.0352, 1.352, 0.8336, 0.3168, 1,
			1.1296, 0.324, 0.3888, 1.0648, 1.648, 1.3664, 0.5832, 1,
			1.1792, 0.448, 0.5376, 1.0896, 1.896, 1.8128, 0.8064, 1,
			1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1;

	Eigen::MatrixXd traj = mr::JointTrajectory(thetastart, thetaend, Tf, N, method);
	cout << "JointTrajectoryTest" << endl
	     << "The result is:" << endl << traj << endl
	     << "It should be:" << endl << result << endl;
}

void ScrewTrajectoryTest()
{
	Eigen::MatrixXd Xstart(4, 4);
	Xstart << 1, 0, 0, 1,
			0, 1, 0, 0,
			0, 0, 1, 1,
			0, 0, 0, 1;
	Eigen::MatrixXd Xend(4, 4);
	Xend << 0, 0, 1, 0.1,
			1, 0, 0, 0,
			0, 1, 0, 4.1,
			0, 0, 0, 1;
	double Tf = 5.0;
	int N = 4;
	int method = 3;

	//正确结果
	std::vector<Eigen::MatrixXd> result(N);
	result[0] = Xstart;
	Eigen::Matrix4d X12;
	X12 << 0.904, -0.25, 0.346, 0.441,
			0.346, 0.904, -0.25, 0.529,
			-0.25, 0.346, 0.904, 1.601,
			0, 0, 0, 1;
	Eigen::Matrix4d X23;
	X23 << 0.346, -0.25, 0.904, -0.117,
			0.904, 0.346, -0.25, 0.473,
			-0.25, 0.904, 0.346, 3.274,
			0, 0, 0, 1;
	result[1] = X12;
	result[2] = X23;
	result[3] = Xend;

	std::vector<Eigen::MatrixXd> traj = mr::ScrewTrajectory(Xstart, Xend, Tf, N, method);

	cout << "ScrewTrajectoryTest" << endl;
	for (int i = 0; i < N; ++i)
	{
		cout << "The result is:" << endl << traj[i] << endl
		     << "It should be:" << endl << result[i] << endl;
	}
}

void CartesianTrajectoryTest()
{
	Eigen::MatrixXd Xstart(4, 4);
	Xstart << 1, 0, 0, 1,
			0, 1, 0, 0,
			0, 0, 1, 1,
			0, 0, 0, 1;
	Eigen::MatrixXd Xend(4, 4);
	Xend << 0, 0, 1, 0.1,
			1, 0, 0, 0,
			0, 1, 0, 4.1,
			0, 0, 0, 1;
	double Tf = 5.0;
	int N = 4;
	int method = 5;

	//正确结果
	std::vector<Eigen::MatrixXd> result(N);
	result[0] = Xstart;
	Eigen::Matrix4d X12;
	X12 << 0.937, -0.214, 0.277, 0.811,
			0.277, 0.937, -0.214, 0,
			-0.214, 0.277, 0.937, 1.651,
			0, 0, 0, 1;
	Eigen::Matrix4d X23;
	X23 << 0.277, -0.214, 0.937, 0.289,
			0.937, 0.277, -0.214, 0,
			-0.214, 0.937, 0.277, 3.449,
			0, 0, 0, 1;
	result[1] = X12;
	result[2] = X23;
	result[3] = Xend;

	std::vector<Eigen::MatrixXd> traj = mr::CartesianTrajectory(Xstart, Xend, Tf, N, method);

	cout << "CartesianTrajectoryTest" << endl;
	for (int i = 0; i < N; ++i)
	{
		cout << "The result is:" << endl << traj[i] << endl
		     << "It should be:" << endl << result[i] << endl;
	}
}

/*--------------------第11章 机器人控制 P287--------------------*/

void ComputedTorqueTest()
{
	//3连杆机械臂算例
	//角度、速度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;
	Eigen::VectorXd eint(3);
	eint << 0.2, 0.2, 0.2;

	//g为在基座坐标系中的重力向量
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

	Eigen::VectorXd thetalistd(3);
	thetalistd << 1.0, 1.0, 1.0;
	Eigen::VectorXd dthetalistd(3);
	dthetalistd << 2, 1.2, 2;
	Eigen::VectorXd ddthetalistd(3);
	ddthetalistd << 0.1, 0.1, 0.1;
	double Kp = 1.3;
	double Ki = 1.2;
	double Kd = 1.1;

	Eigen::VectorXd taulist = mr::ComputedTorque(thetalist, dthetalist, eint, g,
	                                             Mlist, Glist, Slist, thetalistd, dthetalistd, ddthetalistd, Kp, Ki, Kd);

	//正确结果
	Eigen::VectorXd result(3);
	result << 133.00525246, -29.94223324, -3.03276856;

	cout << "ComputedTorqueTest" << endl
	     << "The result is:" << endl << taulist << endl
	     << "It should be:" << endl << result << endl;
}

void SimulateControlTest()
{
	//3连杆机械臂算例
	//角度、速度
	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;
	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;

	//g为在基座坐标系中的重力向量
	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	//Mlist中的每一个M为坐标系{i-1}在{i}中的位形
	std::vector<Eigen::MatrixXd> Mlist;
	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.089159,
			0, 0, 0, 1;
	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
			0, 1, 0, 0.13585,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
			0, 1, 0, -0.1197,
			0, 0, 1, 0.395,
			0, 0, 0, 1;
	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.14225,
			0, 0, 0, 1;
	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	//Glist中的每一个G为空间惯量矩阵，为对角阵，使用asDiagonal
	std::vector<Eigen::MatrixXd> Glist;
	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	//Slist中的每一列Si为旋量轴在基座坐标系{0}中的表示
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
			0, 1, 0, -0.089, 0, 0,
			0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();
	double dt = 0.01;
	Eigen::VectorXd thetaend(3);
	thetaend << M_PI / 2, M_PI / 2, M_PI / 2;
	double Tf = 1.0;
	int N = int(1.0*Tf / dt);
	int method = 5;

	Eigen::MatrixXd traj = mr::JointTrajectory(thetalist, thetaend, Tf, N, method);
	Eigen::MatrixXd thetamatd = traj;
	Eigen::MatrixXd dthetamatd = Eigen::MatrixXd::Zero(N, 3);
	Eigen::MatrixXd ddthetamatd = Eigen::MatrixXd::Zero(N, 3);
	dt = Tf / (N - 1.0);
	for (int i = 0; i < N - 1; ++i)
	{
		dthetamatd.row(i + 1) = (thetamatd.row(i + 1) - thetamatd.row(i)) / dt;
		ddthetamatd.row(i + 1) = (dthetamatd.row(i + 1) - dthetamatd.row(i)) / dt;
	}

	Eigen::VectorXd gtilde(3);
	gtilde << 0.8, 0.2, -8.8;

	std::vector<Eigen::MatrixXd> Mtildelist;
	std::vector<Eigen::MatrixXd> Gtildelist;
	Eigen::Matrix4d Mhat01;
	Mhat01 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.1,
			0, 0, 0, 1;
	Eigen::Matrix4d Mhat12;
	Mhat12 << 0, 0, 1, 0.3,
			0, 1, 0, 0.2,
			-1, 0, 0, 0,
			0, 0, 0, 1;
	Eigen::Matrix4d Mhat23;
	Mhat23 << 1, 0, 0, 0,
			0, 1, 0, -0.2,
			0, 0, 1, 0.4,
			0, 0, 0, 1;
	Eigen::Matrix4d Mhat34;
	Mhat34 << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0.2,
			0, 0, 0, 1;
	Mtildelist.push_back(Mhat01);
	Mtildelist.push_back(Mhat12);
	Mtildelist.push_back(Mhat23);
	Mtildelist.push_back(Mhat34);

	Eigen::VectorXd Ghat1(6);
	Ghat1 << 0.1, 0.1, 0.1, 4, 4, 4;
	Eigen::VectorXd Ghat2(6);
	Ghat2 << 0.3, 0.3, 0.1, 9, 9, 9;
	Eigen::VectorXd Ghat3(6);
	Ghat3 << 0.1, 0.1, 0.1, 3, 3, 3;
	Gtildelist.push_back(Ghat1.asDiagonal());
	Gtildelist.push_back(Ghat2.asDiagonal());
	Gtildelist.push_back(Ghat3.asDiagonal());
	Eigen::MatrixXd Ftipmat = Eigen::MatrixXd::Ones(N, 6);
	double Kp = 20.0;
	double Ki = 10.0;
	double Kd = 18.0;
	int intRes = 8;

	int numTest = 3;  // test 0, N/2-1, N-1 indices of results

	//正确结果
	Eigen::MatrixXd result_taumat(numTest, 3);
	Eigen::MatrixXd result_thetamat(numTest, 3);

	Eigen::VectorXd tau_timestep_beg(3);
	tau_timestep_beg << -14.2640765, -54.06797429, -11.265448;
	Eigen::VectorXd tau_timestep_mid(3);
	tau_timestep_mid << 31.98269367, 9.89625811, 1.47810165;
	Eigen::VectorXd tau_timestep_end(3);
	tau_timestep_end << 57.04391384, 4.75360586, -1.66561523;
	result_taumat << tau_timestep_beg.transpose(),
			tau_timestep_mid.transpose(),
			tau_timestep_end.transpose();

	Eigen::VectorXd theta_timestep_beg(3);
	theta_timestep_beg << 0.10092029, 0.10190511, 0.10160667;
	Eigen::VectorXd theta_timestep_mid(3);
	theta_timestep_mid << 0.85794085, 1.55124503, 2.80130978;
	Eigen::VectorXd theta_timestep_end(3);
	theta_timestep_end << 1.56344023, 3.07994906, 4.52269971;
	result_thetamat << theta_timestep_beg.transpose(),
			theta_timestep_mid.transpose(),
			theta_timestep_end.transpose();

	std::vector<Eigen::MatrixXd> controlTraj = mr::SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, Glist, Slist, thetamatd, dthetamatd,
	                                                               ddthetamatd, gtilde, Mtildelist, Gtildelist, Kp, Ki, Kd, dt, intRes);
	Eigen::MatrixXd traj_tau = controlTraj.at(0);
	Eigen::MatrixXd traj_theta = controlTraj.at(1);
	Eigen::MatrixXd traj_tau_timestep(numTest, 3);
	traj_tau_timestep << traj_tau.row(0),
			traj_tau.row(int(N / 2) - 1),
			traj_tau.row(N - 1);
	Eigen::MatrixXd traj_theta_timestep(numTest, 3);
	traj_theta_timestep << traj_theta.row(0),
			traj_theta.row(int(N / 2) - 1),
			traj_theta.row(N - 1);

	cout << "SimulateControlTest" << endl
	     << "The result is:" << endl << traj_tau_timestep << endl
	     << "It should be:" << endl << result_taumat << endl
	     << "The result is:" << endl << traj_theta_timestep << endl
	     << "It should be:" << endl << result_thetamat << endl;
}

/*--------------------其他--------------------*/

void UR5Test()
{
	//P90 UR5-6R机械臂
	//机器人参数
	float L1 = 0.425;
	float L2 = 0.392;
	float W1 = 0.109;
	float W2 = 0.082;
	float H1 = 0.089;
	float H2 = 0.095;

	//初始值
	Eigen::MatrixXd M(4, 4);
	Eigen::Matrix3d M_R(rotx(-M_PI/2)*rotz(M_PI));
	Eigen::Vector3d M_p(L1+L2,W1+W2,H1-H2);
	M << M_R,     M_p,
			0, 0, 0, 1;
	cout << "初始值:" << endl << "M = " << endl << M << endl;

	//空间坐标系下的关节旋量
	//角速度方向
	Eigen::Vector3d w1(0, 0, 1),
			w2(0, 1, 0),
			w3(0, 1, 0),
			w4(0, 1, 0),
			w5(0, 0, -1),
			w6(0, 1, 0);
	//螺旋轴位置
	Eigen::Vector3d q1(0, 0, 0),
			q2(0, 0, H1),
			q3(L1, 0, H1),
			q4(L1+L2, 0, H1),
			q5(L1+L2, W1, 0),
			q6(L1+L2, 0, H1-H2);

	Eigen::Vector3d v1(-w1.cross(q1)),
			v2(-w2.cross(q2)),
			v3(-w3.cross(q3)),
			v4(-w4.cross(q4)),
			v5(-w5.cross(q5)),
			v6(-w6.cross(q6));
	Eigen::MatrixXd Slist(6, 6);
	Slist << w1, w2, w3, w4, w5, w6,
			v1, v2, v3, v4, v5, v6;
	cout << "空间坐标系下的关节旋量:" << endl << "Slist = " << endl << Slist << endl;

	//关节角度值
	Eigen::VectorXd thetalist(6, 1);
	thetalist << 0,
			-M_PI/4,
			M_PI/4,
			0,
			0,
			0;
	cout << "关节角度值:" << endl << "thetalist = " << endl << thetalist << endl;

	//正运动学
	Eigen::MatrixXd T(4, 4); //目标位姿矩阵
	T = FKinSpace(M, Slist, thetalist);
	cout << "正运动学-末端齐次变换矩阵:" << endl << "T = " << endl << T << endl;

	//雅可比
	Eigen::MatrixXd Js(6, 6);
	Js = JacobianSpace(Slist, thetalist);
	cout << "空间雅可比矩阵:" << endl << "Js = " << endl << Js << endl;

	double eomg = 0.0;
	double ev = 0.0;
	Eigen::VectorXd thetalist0(6, 1);
	thetalist0 << 0,
			-M_PI/5,
			M_PI/5,
			0,
			0,
			0;
	IKinSpace(Slist, M, T, thetalist0, eomg, ev);
	cout << "逆运动学-关节角度值:" << endl << "thetalist = " << endl << thetalist0 << endl;
}