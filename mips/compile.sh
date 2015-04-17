export LD_LIBRARY_PATH=/home/staff/rodolfo/mc723/systemc/lib-linux64:$LD_LIBRARY_PATH
export PATH=/home/staff/rodolfo/mc723/archc/bin:/home/staff/rodolfo/compilers/bin:$PATH

acsim mips.ac -abi
make clean
make

