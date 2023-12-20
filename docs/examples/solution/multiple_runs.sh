#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash

sleep 2
for i in {1..5}
do
    python3 main.py --staged --search_pattern "ParallelSweep" --tag "ParallelSweep"
    sleep 10
done

for i in {1..5}
do
    python3 main.py --staged --search_pattern "ArchimedeanSpiral" --tag "ArchimedeanSpiral"
    sleep 10
done

for i in {1..5}
do
    python3 main.py --staged --search_pattern "RandomWalkCoverage" --tag "RandomWalkCoverage"
    sleep 10
done

for i in {1..5}
do
    python3 main.py --staged --search_pattern "RandomWalk" --tag "RandomWalk"
    sleep 10
done