Generate new report:

```
sudo nsys profile --trace=cuda,nvtx,osrt --output=/home/nicolas/dev/research/KPAX/NSight/review --force-overwrite=true /home/nicolas/dev/research/KPAX/build/executionTimeMain
```

Look at new report:
nsys stats --force-export=true /home/nicolas/dev/research/KPAX/NSight/review.nsys-rep | grep -v "SKIPPED"

Look at Baseline:

```
nsys stats /home/nicolas/dev/research/KPAX/NSight/baseline.nsys-rep | grep -v "SKIPPED"
```