import json
import matplotlib.pyplot as plt
import pickle
import os
import datetime

now = datetime.datetime.now()
folder = now.strftime("%Y-%m-%d-%H-%M-%S")
folder = 'Phase2NoAsync/plots/' + folder
if not os.path.exists(folder):
    os.makedirs(folder)

with open('Phase2NoAsync\dataBase.json', 'r') as file:
    data = json.load(file)

time = [entry['time_us']/1e6 for entry in data['telemetry_history']]
angular_vel_command_x = [entry['angular_vel_command'][0] for entry in data['telemetry_history']]
angular_vel_command_y = [entry['angular_vel_command'][1] for entry in data['telemetry_history']]
angular_vel_command_z = [entry['angular_vel_command'][2] for entry in data['telemetry_history']]
gyr_x = [entry['gyro'][0] for entry in data['telemetry_history']]
gyr_y = [entry['gyro'][1] for entry in data['telemetry_history']]
gyr_z = [entry['gyro'][2] for entry in data['telemetry_history']]
roll_command = [entry['rpy_command'][0] for entry in data['telemetry_history']]
pitch_command = [entry['rpy_command'][1] for entry in data['telemetry_history']]
yaw_command = [entry['rpy_command'][2] for entry in data['telemetry_history']]
orientation_roll = [entry['orientation_rpy_leveled'][0] for entry in data['telemetry_history']]
orientation_pitch = [entry['orientation_rpy_leveled'][1] for entry in data['telemetry_history']]
orientation_yaw = [entry['orientation_rpy_leveled'][2] for entry in data['telemetry_history']]

kalman_delay = [entry['kalman_delay_s'][0] for entry in data['telemetry_history']]
kalman_delay_max = [entry['kalman_delay_s'][1] for entry in data['telemetry_history']]

close_loop_delay = [entry['close_loop_delay_s'][0] for entry in data['telemetry_history']]
close_loop_delay_max = [entry['close_loop_delay_s'][1] for entry in data['telemetry_history']]

stream_delay = [entry['stream_delay_s'][0] for entry in data['telemetry_history']]
stream_delay_max = [entry['stream_delay_s'][1] for entry in data['telemetry_history']]

motors_speed = [entry['motor_speeds'] for entry in data['telemetry_history']]

batteries_voltage = [entry['battery_voltages'] for entry in data['telemetry_history']if 'battery_voltages' in entry]
batteries_lvl = [entry['battery_lvl'] for entry in data['telemetry_history']if 'battery_lvl' in entry]
battery_time = [entry['time_us']/1e6 for entry in data['telemetry_history']if 'battery_lvl' in entry]

# Plot et enregistrement des vitesses angulaires
plt.figure(figsize=(12, 10))
plt.subplot(3, 1, 1)
plt.plot(time, angular_vel_command_x, label='Angular Velocity X command')
plt.plot(time, gyr_x, label='Angular Velocity X')
plt.title('Angular Velocity X Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, angular_vel_command_y, label='Angular Velocity Y command')
plt.plot(time, gyr_y, label='Angular Velocity Y')
plt.title('Angular Velocity Y Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, angular_vel_command_z, label='Angular Velocity Z command')
plt.plot(time, gyr_z, label='Angular Velocity Z')
plt.title('Angular Velocity Z Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()

plt.tight_layout()

# Enregistrement de la figure en tant qu'objet Matplotlib
with open(folder + '/Angular_Velocity_Over_Time.matplotlib', 'wb') as f:
    pickle.dump(plt.gcf(), f)

# Enregistrement de la figure au format PNG
plt.savefig(folder + '/Angular_Velocity_Over_Time.png')

# Affichage de la figure
plt.show()

# Plot et enregistrement des roll et pitch
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(time, roll_command, label='Roll Command')
plt.plot(time, orientation_roll, label='Roll')
plt.title('Roll Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Orientation (radians)')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, pitch_command, label='Pitch Command')
plt.plot(time, orientation_pitch, label='Pitch')
plt.title('Pitch Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Orientation (radians)')
plt.legend()

plt.tight_layout()

# Enregistrement de la figure en tant qu'objet Matplotlib
with open(folder + '/Roll_and_Pitch_Over_Time.matplotlib', 'wb') as f:
    pickle.dump(plt.gcf(), f)

# Enregistrement de la figure au format PNG
plt.savefig(folder + '/Roll_and_Pitch_Over_Time.png')

# Affichage de la figure
plt.show()

# Plot et enregistrement des tâches de retard
plt.figure(figsize=(12, 6))
plt.subplot(3, 1, 1)
plt.plot(time, kalman_delay, label='Kalman Delay')
plt.plot(time, kalman_delay_max, label='Kalman Delay Max')
plt.title('Kalman Delay Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Delay (s)')
plt.ylim(bottom=0)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, close_loop_delay, label='Close Loop Delay')
plt.plot(time, close_loop_delay_max, label='Close Loop Delay Max')
plt.title('Close Loop Delay Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Delay (s)')
plt.ylim(bottom=0)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, stream_delay, label='Stream Delay')
plt.plot(time, stream_delay_max, label='Stream Delay Max')
plt.title('Stream Delay Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Delay (s)')
plt.ylim(bottom=0)
plt.legend()

plt.tight_layout()

# Enregistrement de la figure en tant qu'objet Matplotlib
with open(folder + '/Task_Delay_Over_Time.matplotlib', 'wb') as f:
    pickle.dump(plt.gcf(), f)

# Enregistrement de la figure au format PNG
plt.savefig(folder + '/Task_Delay_Over_Time.png')

# Affichage de la figure
plt.show()

# Plot et enregistrement des vitesses des moteurs
plt.figure(figsize=(12, 6))
plt.plot(time, motors_speed)
plt.title('Motors Speed Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Speed')
plt.legend(['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'])
plt.tight_layout()

# Enregistrement de la figure en tant qu'objet Matplotlib
with open(folder + '/Motors_Speed_Over_Time.matplotlib', 'wb') as f:
    pickle.dump(plt.gcf(), f)

# Enregistrement de la figure au format PNG
plt.savefig(folder + '/Motors_Speed_Over_Time.png')

# Affichage de la figure
plt.show()

# Plot et enregistrement des tensions des batteries
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(battery_time, batteries_voltage)
plt.title('Batteries Voltage Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend(['cell1', 'cell2', 'cell3', 'all cells'])

plt.subplot(2, 1, 2)
plt.plot(battery_time, batteries_lvl)
plt.title('Batteries Level Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Level (%)')
plt.legend(['cell1', 'cell2', 'cell3', 'all cells'])
plt.tight_layout()

# Enregistrement de la figure en tant qu'objet Matplotlib
with open(folder + '/Batteries_Over_Time.matplotlib', 'wb') as f:
    pickle.dump(plt.gcf(), f)

# Enregistrement de la figure au format PNG
plt.savefig(folder + '/Batteries_Over_Time.png')

# Affichage de la figure
plt.show()
