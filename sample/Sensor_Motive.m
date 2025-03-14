function Sensor = Sensor_Motive(rigid_id,initial_yaw_angle,motive)
%% sensor class demo : constructor
% sensor property をSensor classのインスタンス配列として定義
% rpos : RnagePos_sim
Sensor.Flag = struct('Noise',0,'Occlusion', 0); % '1' : Active, '0' : none
Sensor.ObjFeature=4;
Sensor.LocalX     = [ 0.075, -0.075,  0.015;  -0.075, -0.075, -0.015;-0.075,  0.075,  0.015;0.075,  0.075, -0.015];
Sensor.LPF_T=10;
Sensor.initial_yaw_angle = initial_yaw_angle;
% X, Y. Z
Sensor.rigid_id=rigid_id;
Sensor.motive = motive;
end
