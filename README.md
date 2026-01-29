# **Electric Vehicle**

## Overview

This project supplies a basic PID control system utilizing Encoder feedback to help differential drive rc cars
that are powered by Arduino to drive in a straight line and turn 90 degrees.

## How It Works

Inside of the project there is a PID class that uses a PID correction equation to be able to ensure that motors go at the desired speed.
A PID controller utilizes 3 seperate terms: proportional, integral, and derivative. These are used in the PID equation to create a feedback loop to ensure the motor stays close to the target speed.
However, generally the D term is not needed and normally, for this purpose, a simple PI controller should be enough. 
