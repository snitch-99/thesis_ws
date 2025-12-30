# Selective Mesh Refinement for Robotic Mapping

This repository contains the work for the thesis on "Selective Mesh Refinement".

## Overview
The goal of this project is to develop a framework for updating a coarse prior mesh model using high-resolution local point cloud data collected by a robotic platform (e.g., a drone). The core idea is to transmit only the significant geometric updates (discrepancies) to a base station, respecting bandwidth constraints.

## Structure
- **Documents/**: Contains the thesis proposal and related documentation.
- **src/**: Source code for the robotic implementation (ROS 2 packages).
    - `drone_mapping`: Package for drone-based point cloud collection and control.
        - `models/entities`: Contains simulation objects (e.g., rocks).
        - `models/agents`: Contains drone configurations.
