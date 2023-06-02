
echo "start check trajectory precision"

conda activate EVO

evo_traj euroc data.csv --save_as_tum
evo_traj euroc result.csv --save_as_tum

evo_ape tum data.tum result.tum -p -a

echo "end check trajectory precision"
