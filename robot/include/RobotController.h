/*!
 * @file RobotController.h
 * @brief Parent class of user robot controllers.
 * This is an interface between the control code and the common hardware code
 * 用户机器人控制器父类
 * 控制代码和通用硬件代码之间的接口
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/DesiredStateCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/GamepadCommand.h"

/*!
 * Parent class of user robot controllers
 * 用户控制机器人的父类
 */
class RobotController
{
	friend class RobotRunner;
public:
	RobotController()
	{}

	virtual ~RobotController()
	{}

	virtual void initializeController() = 0;
	/**
	 * Called one time every control loop 
	 * 每次循环调用一次
	 */
	virtual void runController() = 0;
	virtual void updateVisualization() = 0;
	virtual ControlParameters* getUserControlParameters() = 0;
	virtual void Estop() 
	{}

protected:
	// 四足数据
	Quadruped<float>* _quadruped = nullptr;
	// 运动学树
	FloatingBaseModel<float>* _model = nullptr;
	// 腿部控制
	LegController<float>* _legController = nullptr;
	// 状态估计器
	StateEstimatorContainer<float>* _stateEstimator = nullptr;
	// 估计出来的数据
	StateEstimate<float>* _stateEstimate = nullptr;
	// 
	GamepadCommand* _driverCommand = nullptr;
	RobotControlParameters* _controlParameters = nullptr;
	DesiredStateCommand<float>* _desiredStateCommand = nullptr;

	VisualizationData* _visualizationData = nullptr;
	RobotType _robotType;
};

#endif
