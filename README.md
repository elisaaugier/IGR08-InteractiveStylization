# IGR08-InteractiveStylization

 <img width="894" height="233" alt="IMG readme" src="https://github.com/user-attachments/assets/31f6b000-e0dc-43e5-ada7-9b173c4fb02b" />


## Project Description

This project is an interactive 3D mesh stylization system developed in C++ with OpenGL and libigl.
Its goal is to let users transform a base mesh into a stylized version through direct interaction, instead of relying only on offline batch processing.

The application provides a real-time workflow where you can load a mesh, place and edit constraints, and iteratively optimize the geometry while immediately seeing the result in the viewer.
The stylization process combines geometric constraints and normal-driven deformation strategies to preserve structure while pushing the shape toward a chosen style.

In practice, the tool supports:

- loading meshes from the local mesh set,
- interactive editing with constraint placement and parameter tuning,
- iterative optimization directly in the UI,
- and exporting the final stylized mesh as an OBJ file.

This repository contains several historical or intermediate folders, but the complete integrated pipeline is in `final-code`, which is the only folder required to build and run the project.
