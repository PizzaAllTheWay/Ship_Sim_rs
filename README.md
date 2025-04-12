# Ship_Sim
A simple ship simulator made with rust

Video

Picture of system: 
speed, x, dx -> Interface -> force thruste, wind params, current params
wind params, current params -> External Forces -> wind_speed, current_speed
force thruste, wind_speed, current_speed -> Ship -> speed, x, dx

# Temp
For debugging run:
./run.sh 2>&1 | tee "log_external_forces.txt"
