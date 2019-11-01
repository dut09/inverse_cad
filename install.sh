# Download CGAL.
if [ ! -d "external/cgal-4.14.1" ]
then
    wget https://github.com/CGAL/cgal/archive/releases/CGAL-4.14.1.zip
    unzip CGAL-4.14.1.zip -d external/
    mv external/cgal-releases-CGAL-4.14.1 external/cgal-4.14.1
    rm -rf "CGAL-4.14.1.zip"
fi

mkdir -p build
cd build
cmake ../
make -j4