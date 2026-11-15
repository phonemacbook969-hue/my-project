# Ikki valli beton qozoni, 2 m³ (3000/2000 L)

Parametrik CAD modeli va ichida yuriladigan 3D ko'ruvchi.

- `cad/build_mixer.py` — CadQuery (OpenCascade B-rep yadrosi) skripti. Barcha o'lchamlar fayl boshidagi parametrlarda.
- `out/beton_qozon_2m3.step` — yig'ma model; SolidWorks, Inventor, Fusion, FreeCAD da ochiladi.
- `viewer/` — Three.js ko'ruvchi (`index.html` + `mixer.json` + `mixer.bin`).

Qayta qurish: `pip install cadquery && python3 cad/build_mixer.py`

Ko'ruvchini mahalliy ochish: `cd viewer && python3 -m http.server` (index.html — sahifa bo'lagi, nashr paytida HTML qobig'iga o'raladi).
