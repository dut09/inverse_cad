sudo apt-get install libcgal-dev
sudo apt-get install swig

cd cpp/core/
swig -c++ -python scene.i
cd ../../

mkdir -p build
cd build
cmake ../
make -j4

cd ..
cp cpp/core/scene.py .
cp build/libscene.so _scene.so
