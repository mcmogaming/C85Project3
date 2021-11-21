aclocal
automake --add-missing --include-deps
autoconf
./configure
make
sudo ./src/roboSoccer /dev/video2 0 2