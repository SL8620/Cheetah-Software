/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * 运行机器人控制器的通用框架
 * 该代码是四足的控制代码和硬件/sim之间的通用接口
 */

#include <unistd.h>

#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

// 构造函数 将运行框架加入任务管理器
// 在HardwareBridge中使用
// 实例化运行器，传入：控制器，任务管理器，参数，名称
RobotRunner::RobotRunner(RobotController* robot_ctrl, PeriodicTaskManager* manager, float period, std::string name)
			: PeriodicTask(manager, period, name),	// 添加任务
			 _lcm(getLcmUrl(255)) 
{
	_robot_ctrl = robot_ctrl;
}

/**
 * 初始化机器人模型，状态估计器，腿部控制器，机器人数据，以及任何控制逻辑特定的数据
 */
void RobotRunner::init() 
{
	printf("[RobotRunner] initialize\n");

	// Build the appropriate Quadruped object
	if (robotType == RobotType::MINI_CHEETAH) 
	{
		_quadruped = buildMiniCheetah<float>();
	} 
	else 
	{
		_quadruped = buildCheetah3<float>();
	}

	// 初始化模型和机器人数据
	_model = _quadruped.buildModel();	// 机器人模型
	_jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);	// 开机时腿的位置

	// 始终初始化腿控制器和状态估计器
	_legController = new LegController<float>(_quadruped);	// 初始化腿控制器
	// 构造状态估计器，传入参数指针或数值
	_stateEstimator = new StateEstimatorContainer<float>(cheaterState, vectorNavData, _legController->datas,&_stateEstimate, controlParameters);
	// 重置状态估计，定义在文件最后
	initializeStateEstimator(false);

	memset(&rc_control, 0, sizeof(rc_control_settings));
	// 初始化DesiredStateCommand对象
	_desiredStateCommand = new DesiredStateCommand<float>(	driverCommand,
															&rc_control,
															controlParameters,
															&_stateEstimate,
															controlParameters->controller_dt);

	// 控制器初始化
	_robot_ctrl->_model = &_model;						// 模型
	_robot_ctrl->_quadruped = &_quadruped;				// 机器人的物理特性表征
	_robot_ctrl->_legController = _legController;		// 腿部控制器，对象	
	_robot_ctrl->_stateEstimator = _stateEstimator;		// 状态估计器
	_robot_ctrl->_stateEstimate = &_stateEstimate;		// 状态估计值
	_robot_ctrl->_visualizationData= visualizationData;	// 可视化数据
	_robot_ctrl->_robotType = robotType;				// 机器人类型
	_robot_ctrl->_driverCommand = driverCommand;		// 驱动命令
	_robot_ctrl->_controlParameters = controlParameters;// 控制参数	
	_robot_ctrl->_desiredStateCommand = _desiredStateCommand;	// 期望状态命令

	_robot_ctrl->initializeController();		// 初始化控制器

}

/**
 * 通过调用每个主要组件运行各自步骤，进而运行整个机器人系统
 * 机器人系统每个执行周期的运行内容（运行周期从参数文件获取）
 */
void RobotRunner::run() 
{
	// 运行状态预测
	_stateEstimator->run();

	//可视化数据清零
	visualizationData->clear();

	// 更新来自机器人的数据到LegController对象
	setupStep();

	static int count_ini(0);
	++count_ini;
	// 计数50次后开启腿部控制
	if (count_ini < 10) 
	{
		_legController->setEnabled(false);
	}
	else if (20 < count_ini && count_ini < 30) 
	{
		_legController->setEnabled(false);
	} 
	else if (40 < count_ini && count_ini < 50) 
	{
		_legController->setEnabled(false);
	} 
	else 
	{
		// 腿部电机使能
		_legController->setEnabled(true);

		// 当遥控器控制时，如rc_control.mode == 0,则会把LegController对象的控制命令数据清零
		if( (rc_control.mode == 0) && controlParameters->use_rc ) 
		{
			if(count_ini%1000 ==0)   printf("ESTOP!\n");
			for (int leg = 0; leg < 4; leg++) 
			{
				_legController->commands[leg].zero();
			}
			// ESTOP调用了_controlFSM->initialize()
			_robot_ctrl->Estop();
		}
		else 
		{
			// 如果关节位置初始对象没有初始化时，将LegController对象的各个关节pd参数赋值
			if (!_jpos_initializer->IsInitialized(_legController)) 
			{
				Mat3<float> kpMat;
				Mat3<float> kdMat;
				// 更新jpos反馈增益
				if (robotType == RobotType::MINI_CHEETAH) 
				{
					kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
					kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
				} 
				else if (robotType == RobotType::CHEETAH_3) 
				{
					kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
					kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
				} 
				else 
				{
					assert(false);
				} 

				for (int leg = 0; leg < 4; leg++) 
				{
					_legController->commands[leg].kpJoint = kpMat;
					_legController->commands[leg].kdJoint = kdMat;
				}
			} 
			else 
			{
				// 初始化完成，执行机器人控制器

				// 运行控制
				_robot_ctrl->runController();
				cheetahMainVisualization->p = _stateEstimate.position;

				// 更新可视化
				_robot_ctrl->updateVisualization();
				cheetahMainVisualization->p = _stateEstimate.position;
			}
		}

	}

	// 可视化
	for (int leg = 0; leg < 4; leg++) 
	{
		for (int joint = 0; joint < 3; joint++) 
		{
		cheetahMainVisualization->q[leg * 3 + joint] = _legController->datas[leg].q[joint];
		}
	}
	cheetahMainVisualization->p.setZero();
	cheetahMainVisualization->p = _stateEstimate.position;
	cheetahMainVisualization->quat = _stateEstimate.orientation;

	// 将生成的leg controller命令数据更新到各机器人控制系统
	finalizeStep();
}

/*!
 * 在运行用户代码之前，设置legController和估计器
 */
void RobotRunner::setupStep() 
{
	// 选择机型，更新腿数据
	if (robotType == RobotType::MINI_CHEETAH) 
	{
		_legController->updateData(spiData);
	} 
	else if (robotType == RobotType::CHEETAH_3) 
	{
		_legController->updateData(tiBoardData);
	} 
	else 
	{
		assert(false);
	}

	// 为新的循环设置_legController
	_legController->zeroCommand();
	_legController->setEnabled(true);
	_legController->setMaxTorqueCheetah3(208.5);

	// 状态估计器
	// check transition to cheater mode:
	if (!_cheaterModeEnabled && controlParameters->cheater_mode) 
	{
		printf("[RobotRunner] Transitioning to Cheater Mode...\n");
		initializeStateEstimator(true);
		// todo any configuration
		_cheaterModeEnabled = true;
	}

	// check transition from cheater mode:
	if (_cheaterModeEnabled && !controlParameters->cheater_mode) 
	{
		printf("[RobotRunner] Transitioning from Cheater Mode...\n");
		initializeStateEstimator(false);
		// todo any configuration
		_cheaterModeEnabled = false;
	}

	// 获取遥控器设置
	get_rc_control_settings(&rc_control);

	// todo safety checks, sanity checks, etc...
}

/*!
 * 在用户代码之后，发送腿命令，更新状态估计，发布调试数据
 */
void RobotRunner::finalizeStep() 
{
	// 选择机型，发布数据
	if (robotType == RobotType::MINI_CHEETAH) 
	{
		_legController->updateCommand(spiCommand);
	} 
	else if (robotType == RobotType::CHEETAH_3) 
	{
		_legController->updateCommand(tiBoardCommand);
	} 
	else 
	{
		assert(false);
	}

	// 发布LCM主题
	_legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
	_stateEstimate.setLcm(state_estimator_lcm);
	_lcm.publish("leg_control_command", &leg_control_command_lcm);
	_lcm.publish("leg_control_data", &leg_control_data_lcm);
	_lcm.publish("state_estimator", &state_estimator_lcm);
	_iterations++;
}

/*!
 * 重置给定模式下的状态估计器
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) 
{
	// 删除所有估计器
	_stateEstimator->removeAllEstimators();
	// 添加状态估计器
	_stateEstimator->addEstimator<ContactEstimator<float>>();
	Vec4<float> contactDefault;
	contactDefault << 0.5, 0.5, 0.5, 0.5;
	_stateEstimator->setContactPhase(contactDefault);
	if (cheaterMode) 
	{
		_stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
		_stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
	} 
	else 
	{
		// 默认使用这个，添加位置角度估计器
		_stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
		_stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
	}
}

// 析构函数，删除所有对象
RobotRunner::~RobotRunner() 
{
	delete _legController;
	delete _stateEstimator;
	delete _jpos_initializer;
}

void RobotRunner::cleanup() 
{}
