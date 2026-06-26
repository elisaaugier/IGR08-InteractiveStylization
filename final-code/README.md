# final-code

This is the only project folder you need for this repository.

## What This Folder Contains

`final-code` is the integrated version of the project (mesh loading, interactive stylization UI, and export).

## Prerequisites

- CMake 3.10+
- A C++17-compatible compiler
	- Windows: Visual Studio Build Tools (MSVC)
	- Linux: GCC or Clang
- OpenGL-capable graphics drivers

## Build

From the `final-code` directory:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

## Run

Important: run the executable from `final-code/build`, so relative mesh paths resolve correctly.

Windows (PowerShell):

```powershell
cd build
.\app.exe
```

If your generator creates a `Release` subfolder, use:

```powershell
cd build
.\Release\app.exe 
```

Linux/macOS:

```bash
cd build
./app 
```

## Basic Controls

- Click: place constrained points
- Space: start/stop optimization
- R: reset
- `<` / `>`: decrease/increase lambda
- Q/W: rotate X
- A/S: rotate Y
- Z/X: rotate Z
- L: rescale

## Output

Use the `save output mesh` button in the UI.
The exported mesh is written to `final-code/output.obj`.