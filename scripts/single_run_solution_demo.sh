#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



# # Flags
# while getopts "rcpa" opt; do
#     case "${opt}" in
#         r)
#             python3 demo.py --staged --search_pattern "RandomWalk" --tag "RandomWalk"
#             ;;
#         c)
#             python3 demo.py --staged --search_pattern "RandomWalkCoverage" --tag "RandomWalkCoverage"
#             ;;
        
#         p)
#             python3 demo.py --staged --search_pattern "ParallelSweep" --tag "ParallelSweep"
#             ;;
#         a)
#             python3 demo.py --staged --search_pattern "ArchimedeanSpiral" --tag "ArchimedeanSpiral"
#             ;;
#         \?)
#             echo "[ERROR] Invalid flag provided"
#             exit
#             ;;
#         *)
#         ;;
#     esac
# done

printf "Solution: \nRandom Walk Covered: 1\nParallel Sweep: 2\nArchimedian Spiral: 3\n"
read SOLUTION_NUM


case $SOLUTION_NUM in

  1)
    SOLUTION=random_walk_covered_demo.sh
    ;;

  2)
    SOLUTION=random_walk_covered_demo.sh
    ;;

  3)
    SOLUTION=random_walk_covered_demo.sh
    ;;

  *)
    echo That is not an option
    exit
    ;;
esac

MACE_CONTAINER_ID=$(docker ps -aqf "name=mace")

cd ../docs/examples/solution

docker exec -i $MACE_CONTAINER_ID bash < $SOLUTION