#!/bin/bash
g++ raytracer.cpp -o raytracer
./raytracer > raytracer.ppm
gimp raytracer.ppm