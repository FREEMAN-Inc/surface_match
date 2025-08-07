git clone --recurse-submodules https://github.com/FREEMAN-Inc/surface_match.git

mkdir release 

cd release

cmake -DCMAKE_BUILD_TYPE=Release ..

make 

./test_ppf small.ply small_seg_0.ply
