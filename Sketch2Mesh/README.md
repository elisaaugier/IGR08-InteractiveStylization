# Sketch2Mesh

Interface interactive de sketch 2D vers primitives 3D.

## Build

Depuis `Sketch2Mesh` :

```powershell
cmake -S . -B .\build "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
cmake --build .\build
```

## Lancement

Depuis `Sketch2Mesh` :

```powershell
.\build\meshSketchRecognizer_bin.exe
```

Ou depuis `Sketch2Mesh\build` :

```powershell
.\meshSketchRecognizer_bin.exe
```

## Utilisation rapide

- Dessiner un trait dans la vue.
- Relacher pour faire apparaitre la primitive reconnue.
- Deplacer un objet avec la souris.
- Ajuster les parametres dans l'interface ImGui.

## Dossiers utiles

- `meshes/` : meshes de test
- `primitives/` : primitives de reference
- `utils/` : fonctions utilitaires
