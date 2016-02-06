# use: gnuplot envelope-graphic.gnu
#
# uncomment for generate image and comment "pause -1"
set terminal postscript eps size 15,9 enhanced color font 'Sans,18' linewidth 2
set output 'envelope-graphic.eps'

set angles degrees
set xrange [-360:1760]
set style line 1 lt 1 lw 2 linecolor rgb "red"
set style line 2 lt 1 lw 1 linecolor rgb "green"
set style line 3 lt 1 lw 1 linecolor rgb "blue"

set style line 102 lc rgb '#d6d7d9' lt 0 lw 1
set grid back ls 102

set samples 3000

a(x) = sin(x)*cos(x/10)
b(x) = sin(x+120)*cos(x/10)
c(x) = sin(x+240)*cos(x/10)


max(x, y) = (x>y?x:y)
min(x, y) = (x<y?x:y)

plot a(x), b(x), c(x), \
     2*sqrt((a(x)**2 - b(x)*c(x))/3), \
     sqrt(((2 * b(x) + a(x))**2)/3 + a(x)**2), \
     sqrt(((2 * c(x) + b(x))**2)/3 + b(x)**2), \
     sqrt(((2 * c(x) + a(x))**2)/3 + a(x)**2), \
     (abs(a(x)) + abs(b(x)) + abs(c(x))) / sqrt(3), \
     ((abs(a(x))) + abs(b(x)) + abs(c(x))) / 3.5 + (max(a(x), b(x)) + max(b(x), c(x)) + max(c(x), a(x))) / 3.5, \
     (max(abs(a(x)), abs(b(x))) + max(abs(b(x)), abs(c(x))) + max(abs(c(x)), abs(a(x))))/2.7 + \
     (min(abs(a(x)), abs(b(x))) + min(abs(b(x)), abs(c(x))) + min(abs(c(x)), abs(a(x))))/15



# uncomment if need wait for keypress and comment two first string with svg
#pause -1
