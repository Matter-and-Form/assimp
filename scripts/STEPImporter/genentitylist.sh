#!/bin/sh
cd ../../code
grep -E 'STEP[A-Za-z][A-Za-z_]*' -o STEPImporter.cpp STEPGeometry.cpp STEPCurve.cpp STEPMaterial.cpp STEPUtil.cpp | sed s/.*:STEP// | tr A-Z a-z | sort | uniq > ../scripts/STEPImporter/output.txt
