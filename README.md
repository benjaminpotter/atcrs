
For generating the GNU parallel input:

Fish:
```
for f in (realpath ./trials/*.json)
    echo $f (string replace -r '\.json$' '.csv' (basename $f))
end
```

Bash:
```bash
for f in "$(realpath ./trials)"/*.json; do
    abs="$(realpath "$f")"
    stem="$(basename "$f" .json)"
    echo "$abs" "$stem.csv"
done
```

Running on CAC:

Per process:
- 20GB mem 
- 2 cores

Machines:
- 486 gb mem
- 64 core
- `CPUBASE_6HRS`

Plan:
- Run -j 24 to use 48 cores and 480 GB mem.
- There are 1978 tasks
- Schedule 3.5 min avg per task
- 1978 tasks * 3.5 min per task / 60 min per hr / 24 tasks = 4.8 hours
- Go with a buffer and schedule 6 hours
- For upper bound 5 min per task + switching we get 6.9 hours
- Worst case we don't complete all of the tasks, but we still get most of the data

E. Tolstaya, A. Ribeiro, V. Kumar and A. Kapoor, "Inverse Optimal Planning for Air Traffic Control," 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Macau, China, 2019, pp. 7535-7542, doi: 10.1109/IROS40897.2019.8968460. keywords: {Airplanes;Costs;Aerospace electronics;Cost function;Airports;Trajectory;Planning;Safety;Frequency measurement;Air traffic control},

