Testing files while under development

For debugging run:
./run.sh 2>&1 | tee "log_external_forces.txt"
./run.sh 2>&1 | tee "log_gnss.txt"