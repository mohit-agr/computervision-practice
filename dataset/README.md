On your machine download and extract EuRoC MAC dataset.

Link : https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

Example directory structure :
<pre>
   -- dataset
      |-- MH_01_easy
          |-- mav0
              |-- cam0
                  |-- data
                      |-- <timestamp>.png
                  |-- data.csv
                  |-- sensor.yaml
              |-- cam1
                  |-- data
                      |-- <timestamp>.png
                  |-- data.csv
                  |-- sensor.yaml
              |-- imu0
                  |-- data.csv
                  |-- sensor.yaml
              |-- leica0
                  |-- data.csv
                  |-- sensor.yaml
              |-- state_groundtruth_estimate0
                  |-- .DS_Store
                  |-- data.csv
                  |-- sensor.yaml
              |-- body.yaml
</pre>
