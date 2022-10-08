#ifndef MRTEST_H
#define MRTEST_H

/*--------------------第3章 刚体运动 P69--------------------*/

void RotInvTest();
void VecToso3Test();
void so3ToVecTest();
void AxisAng3Test();
void MatrixExp3Test();
void MatrixLog3Test();
void DistanceToSO3Test();
void TestIfSO3Test();
void ProjectToSO3Test();
void RpToTransTest();
void TransToRpTest();
void TransInvTest();
void VecTose3Test();
void se3ToVecTest();
void AdjointTest();
void ScrewToAxisTest();
void AxisAng6Test();
void MatrixExp6Test();
void MatrixLog6Test();
void DistanceToSE3Test();
void TestIfSE3Test();
void ProjectToSE3Test();

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
void InverseDynamicsTrajectoryTest();//TODO
void ForwardDynamicsTrajectoryTest();//TODO

/*--------------------第9章 轨迹生成 P216--------------------*/

void CubicTimeScalingTest();//TODO
void QuinticTimeScalingTest();//TODO
void JointTrajectoryTest();//TODO
void ScrewTrajectoryTest();//TODO
void CartesianTrajectoryTest();//TODO

/*--------------------第11章 机器人控制 P287--------------------*/

void ComputedTorqueTest();//TODO
void SimulateControlTest();//TODO

/*--------------------其他--------------------*/
void UR5Test();//TODO

#endif //MRTEST_H
