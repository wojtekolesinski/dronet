#-----------------------------------------------------------#
#           _  _ ___  ___  ___  _  _ ___ ___                #
#          | \| |   \| _ \/ _ \| \| | __/ __|               #
#          | .` | |) |   / (_) | .` | _|\__ \               #
#          |_|\_|___/|_|_\\___/|_|\_|___|___/               #
#                                                           #
#-----------------------------------------------------------#


#test others algorithms
for nd in "5" "10" "15" "20" "25" "30" ;
do
<<<<<<< HEAD
    for alg in  "RND" "GEO" "QMR"
=======
    for alg in  "Q_FANET_OUR" ; #"RND" "GEO" "Q_FANET" "QMR"
>>>>>>> 9d559797d54f248868f8e7783023a14ee0ae425b
    do
        echo "run: ${alg} - ndrones ${nd} "
        python -m src.experiments.experiment_ndrones -nd ${nd} -i_s 0 -e_s 10 -alg ${alg} &
        python -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        python -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
    done;
done;
wait


