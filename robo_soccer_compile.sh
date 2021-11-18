aclocal
automake --add-missing --include-deps
autoconf
./configure
make
sudo ./src/roboSoccer /dev/video0 0 0