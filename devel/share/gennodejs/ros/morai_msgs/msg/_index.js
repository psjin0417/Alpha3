
"use strict";

let ObjectStatusList = require('./ObjectStatusList.js');
let CMDConveyor = require('./CMDConveyor.js');
let RobotState = require('./RobotState.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let ObjectStatus = require('./ObjectStatus.js');
let ReplayInfo = require('./ReplayInfo.js');
let Transforms = require('./Transforms.js');
let VelocityCmd = require('./VelocityCmd.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let IntscnTL = require('./IntscnTL.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let IntersectionControl = require('./IntersectionControl.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let Lamps = require('./Lamps.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let DillyCmd = require('./DillyCmd.js');
let PRStatus = require('./PRStatus.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let WaitForTick = require('./WaitForTick.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let TOF = require('./TOF.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let CollisionData = require('./CollisionData.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let ExternalForce = require('./ExternalForce.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let TrafficLight = require('./TrafficLight.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let RobotOutput = require('./RobotOutput.js');
let GhostMessage = require('./GhostMessage.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let Obstacles = require('./Obstacles.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let VehicleCollision = require('./VehicleCollision.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let ShipState = require('./ShipState.js');
let MapSpec = require('./MapSpec.js');
let SensorPosControl = require('./SensorPosControl.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let Conveyor = require('./Conveyor.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let RadarDetections = require('./RadarDetections.js');
let VehicleSpec = require('./VehicleSpec.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let SaveSensorData = require('./SaveSensorData.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let CtrlCmd = require('./CtrlCmd.js');
let Obstacle = require('./Obstacle.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let EventInfo = require('./EventInfo.js');
let ERP42Info = require('./ERP42Info.js');
let SVADC = require('./SVADC.js');
let WheelControl = require('./WheelControl.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let PREvent = require('./PREvent.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let RadarDetection = require('./RadarDetection.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let GPSMessage = require('./GPSMessage.js');
let GVStateCmd = require('./GVStateCmd.js');

module.exports = {
  ObjectStatusList: ObjectStatusList,
  CMDConveyor: CMDConveyor,
  RobotState: RobotState,
  MoraiSimProcHandle: MoraiSimProcHandle,
  ObjectStatus: ObjectStatus,
  ReplayInfo: ReplayInfo,
  Transforms: Transforms,
  VelocityCmd: VelocityCmd,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  IntscnTL: IntscnTL,
  VehicleSpecIndex: VehicleSpecIndex,
  FaultInjection_Tire: FaultInjection_Tire,
  IntersectionControl: IntersectionControl,
  MultiPlayEventResponse: MultiPlayEventResponse,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  SyncModeRemoveObject: SyncModeRemoveObject,
  Lamps: Lamps,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  SyncModeResultResponse: SyncModeResultResponse,
  EgoVehicleStatus: EgoVehicleStatus,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  SyncModeCmd: SyncModeCmd,
  ShipCtrlCmd: ShipCtrlCmd,
  DillyCmd: DillyCmd,
  PRStatus: PRStatus,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  WaitForTick: WaitForTick,
  MoraiSimProcStatus: MoraiSimProcStatus,
  TOF: TOF,
  NpcGhostCmd: NpcGhostCmd,
  MultiPlayEventRequest: MultiPlayEventRequest,
  WoowaDillyStatus: WoowaDillyStatus,
  CollisionData: CollisionData,
  FaultStatusInfo: FaultStatusInfo,
  FaultInjection_Controller: FaultInjection_Controller,
  ExternalForce: ExternalForce,
  ObjectStatusExtended: ObjectStatusExtended,
  SyncModeCmdResponse: SyncModeCmdResponse,
  DdCtrlCmd: DdCtrlCmd,
  MapSpecIndex: MapSpecIndex,
  TrafficLight: TrafficLight,
  WaitForTickResponse: WaitForTickResponse,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  SkateboardStatus: SkateboardStatus,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SetTrafficLight: SetTrafficLight,
  RobotOutput: RobotOutput,
  GhostMessage: GhostMessage,
  MultiEgoSetting: MultiEgoSetting,
  Obstacles: Obstacles,
  VehicleCollisionData: VehicleCollisionData,
  VehicleCollision: VehicleCollision,
  SyncModeInfo: SyncModeInfo,
  ShipState: ShipState,
  MapSpec: MapSpec,
  SensorPosControl: SensorPosControl,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  GeoVector3Message: GeoVector3Message,
  Conveyor: Conveyor,
  FaultInjection_Response: FaultInjection_Response,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  RadarDetections: RadarDetections,
  VehicleSpec: VehicleSpec,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  SaveSensorData: SaveSensorData,
  ScenarioLoad: ScenarioLoad,
  MoraiTLInfo: MoraiTLInfo,
  ObjectStatusListExtended: ObjectStatusListExtended,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  ManipulatorControl: ManipulatorControl,
  CtrlCmd: CtrlCmd,
  Obstacle: Obstacle,
  IntersectionStatus: IntersectionStatus,
  EventInfo: EventInfo,
  ERP42Info: ERP42Info,
  SVADC: SVADC,
  WheelControl: WheelControl,
  PRCtrlCmd: PRCtrlCmd,
  GVDirectCmd: GVDirectCmd,
  SyncModeSetGear: SyncModeSetGear,
  FaultInjection_Sensor: FaultInjection_Sensor,
  MoraiTLIndex: MoraiTLIndex,
  PREvent: PREvent,
  MoraiSrvResponse: MoraiSrvResponse,
  RadarDetection: RadarDetection,
  SyncModeAddObject: SyncModeAddObject,
  NpcGhostInfo: NpcGhostInfo,
  DillyCmdResponse: DillyCmdResponse,
  GPSMessage: GPSMessage,
  GVStateCmd: GVStateCmd,
};
