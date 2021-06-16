# Curve Fitting using Least Square, Total Least Square and RANSAC

## Files: 
	Ball_travel_10fps.mp4 : input video without noise
	Ball_travel_2_updated.mp4: input video with noise
	curveFitting.py - main file to run for Problem2 - curve fitting
	homography.py - file to run for Problem2
	myUtils.py - helper file

## How to run
	python3 curveFitting.py --vid Ball_travel_10fps.mp4
	python3 curveFitting.py --vid Ball_travel_2_updated.mp4

## Problem 1 - Without Noise
<p align="center">
<img src="Ball_travel_10fps.gif"/>
</p>

### Least Square method
<p align="center">
<img src="Least Square Approximation Without Noise.jpg"/>
</p>

### Total Least Square method
<p align="center">
<img src="Total Least Square Approximation Without Noise.jpg"/>
</p>

### RANSAC
<p align="center">
<img src="RANSAC Without Noise.jpg"/>
</p>


## Problem 2 - With Noise
<p align="center">
<img src="Ball_travel_2_updated.gif"/>
</p>

### Least Square method
<p align="center">
<img src="Least Square Approximation With Noise.jpg"/>
</p>

### Total Least Square method
<p align="center">
<img src="Total Least Square Approximation With Noise.jpg"/>
</p>

### RANSAC
<p align="center">
<img src="RANSAC With Noise.jpg"/>
</p>
