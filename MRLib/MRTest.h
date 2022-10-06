#ifndef MRTEST_H
#define MRTEST_H

/*--------------------第3章 刚体运动 P69--------------------*/

void RotInvTest();
void VecToso3Test();
void so3ToVecTest();//TODO
void AxisAng3Test();//TODO
void MatrixExp3Test();//TODO
void MatrixLog3Test();//TODO
void DistanceToSO3Test();
void TestIfSO3Test();
void ProjectToSO3Test();//TODO
void RpToTransTest();//TODO
void TransToRpTest();//TODO
void TransInvTest();
void VecTose3Test();//TODO
void se3ToVecTest();//TODO
void AdjointTest();
void ScrewToAxisTest();
void AxisAng6Test();
void MatrixExp6Test();//TODO
void MatrixLog6Test();
void DistanceToSE3Test();
void TestIfSE3Test();
void ProjectToSE3Test();//TODO

/*--------------------第4章 正向运动学 P99--------------------*/

void FKinBodyTest();
void FKinSpaceTest();

/*--------------------第5章 一阶运动学与静力学 P125--------------------*/

void JacobianBodyTest();
void JacobianSpaceTest();

/*--------------------第6章 逆运动学 P144--------------------*/

void IKinBodyTest();
void IKinSpaceTest();

/*--------------------第8章 开链动力学 P197--------------------*/

void adTest();
void InverseDynamicsTest();
void MassMatrixTest();
void VelQuadraticForcesTest();
void GravityForcesTest();
void EndEffectorForcesTest();
void ForwardDynamicsTest();
void EulerStepTest();
void InverseDynamicsTrajectoryTest();
void ForwardDynamicsTrajectoryTest();

/*--------------------第9章 轨迹生成 P216--------------------*/

void CubicTimeScalingTest();
void QuinticTimeScalingTest();
void JointTrajectoryTest();
void ScrewTrajectoryTest();
void CartesianTrajectoryTest();

/*--------------------第11章 机器人控制 P287--------------------*/

void ComputedTorqueTest();
void SimulateControlTest();

/*--------------------其他--------------------*/
void UR5Test();

#endif //MRTEST_H
