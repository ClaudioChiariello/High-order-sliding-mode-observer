sed -i 's/#include "tmwtypes.h"/\/\* #include "tmwtypes.h" \*\//g' rtwtypes.h
gcc -O3 -fPIC -shared -I. vehicle_dynamics_numeric.c -o libvehicle_dynamics.so -lm

#QUesti comandi devono essere lanciati nel percorso 'codegen/lib/vehicle_dynamics_numeric'
Il primo serve a commentare un include che non viene usato, e quindi inutile. Il secondo compila la shared library;