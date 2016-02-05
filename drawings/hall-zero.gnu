# use: gnuplot hall-zero.gnu
#
# uncomment for generate image and comment "pause -1"
set terminal postscript eps size 15,9 enhanced color font 'Sans,20' linewidth 2
set output 'hall-zero.eps'

set angles degrees
set xrange [-360:3000]
set style line 1 lt 1 lw 2 linecolor rgb "red"
set style line 2 lt 1 lw 1 linecolor rgb "green"
set style line 3 lt 1 lw 1 linecolor rgb "blue"

set style line 102 lc rgb '#d6d7d9' lt 0 lw 1
set grid back ls 102

set samples 3000

a(x) = sin(x)*sin(x/20)+cos(x/10)
b(x) = sin(x+120)*sin(x/20)+cos(x/10)
c(x) = sin(x+240)*sin(x/20)+cos(x/10)



plot a(x), b(x), c(x), (a(x) + b(x) + c(x)) / 3



# uncomment if need wait for keypress and comment two first string with svg
#pause -1
