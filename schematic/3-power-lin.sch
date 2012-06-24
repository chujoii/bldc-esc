v 20110115 2
C 40000 40000 0 0 0 title-B.sym
C 43100 35400 1 270 0 resistor-2.sym
{
T 43450 35000 5 10 0 0 270 0 1
device=RESISTOR
T 43400 35200 5 10 1 1 270 0 1
refdes=R303
T 43100 35400 5 10 1 1 0 0 1
value=100k
T 43100 35400 5 10 1 1 0 0 1
footprint=0805
}
C 43100 34100 1 270 0 resistor-2.sym
{
T 43450 33700 5 10 0 0 270 0 1
device=RESISTOR
T 43400 33900 5 10 1 1 270 0 1
refdes=R304
T 43100 34100 5 10 1 1 0 0 1
value=22k
T 43100 34100 5 10 1 1 0 0 1
footprint=0805
}
C 43100 32800 1 270 0 resistor-2.sym
{
T 43450 32400 5 10 0 0 270 0 1
device=RESISTOR
T 43400 32600 5 10 1 1 270 0 1
refdes=R305
T 43100 32800 5 10 1 1 0 0 1
value=22k
T 43100 32800 5 10 1 1 0 0 1
footprint=0805
}
C 43100 36700 1 270 0 resistor-2.sym
{
T 43450 36300 5 10 0 0 270 0 1
device=RESISTOR
T 43400 36500 5 10 1 1 270 0 1
refdes=R306
T 43100 36700 5 10 1 1 0 0 1
value=15k
T 43100 36700 5 10 1 1 0 0 1
footprint=0805
}
C 41600 36900 1 0 0 input-2.sym
{
T 41600 37100 5 10 1 1 0 0 1
net=VBUS_D:1
T 42200 37600 5 10 0 0 0 0 1
device=none
T 42100 37000 5 10 0 1 0 7 1
value=INPUT
}
C 45000 34200 1 0 0 output-2.sym
{
T 45900 34400 5 10 1 1 0 0 1
net=VMOT:1
T 45200 34900 5 10 0 0 0 0 1
device=none
T 45900 34300 5 10 0 1 0 1 1
value=OUTPUT
}
C 45000 32900 1 0 0 output-2.sym
{
T 45900 33100 5 10 1 1 0 0 1
net=VMOT_Half:1
T 45200 33600 5 10 0 0 0 0 1
device=none
T 45900 33000 5 10 0 1 0 1 1
value=OUTPUT
}
C 43800 34100 1 270 0 capacitor-1.sym
{
T 44500 33900 5 10 0 0 270 0 1
device=CAPACITOR
T 44300 33900 5 10 1 1 270 0 1
refdes=C307
T 44700 33900 5 10 0 0 270 0 1
symversion=0.1
T 43800 34100 5 10 1 1 0 0 1
value=2.2n
T 43800 34100 5 10 1 1 0 0 1
footprint=0805
}
C 44300 32800 1 270 0 capacitor-1.sym
{
T 45000 32600 5 10 0 0 270 0 1
device=CAPACITOR
T 44800 32600 5 10 1 1 270 0 1
refdes=C308
T 45200 32600 5 10 0 0 270 0 1
symversion=0.1
T 44300 32800 5 10 1 1 0 0 1
value=100n
T 44300 32800 5 10 1 1 0 0 1
footprint=0805
}
C 43100 31200 1 0 0 gnd-1.sym
N 43000 37000 43200 37000 4
N 43200 37000 43200 36700 4
N 43200 35800 43200 35400 4
N 43200 34500 43200 34100 4
N 43200 33200 43200 32800 4
N 43200 31900 43200 31500 4
N 44500 31900 44500 31700 4
N 44500 31700 43200 31700 4
N 44500 32800 44500 33000 4
N 43200 33000 45000 33000 4
N 44000 33200 44000 31700 4
N 44000 34100 44000 34300 4
N 43200 34300 45000 34300 4
B 41100 30900 6100 6700 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
T 41200 31000 9 10 1 0 0 0 1
V measurement
C 46200 62900 1 0 1 MC34063-1.sym
{
T 44400 64300 5 10 1 1 0 0 1
refdes=U302
T 45900 64900 5 10 0 0 0 6 1
footprint=SO8
T 45900 64500 5 10 0 0 0 6 1
device=MC34063
T 45900 65500 5 10 0 0 0 6 1
symversion=1.0
T 44300 64500 5 10 1 1 0 0 1
comment=step-down converter
}
C 45500 61800 1 180 0 resistor-2.sym
{
T 45100 61450 5 10 0 0 180 0 1
device=RESISTOR
T 45300 61500 5 10 1 1 180 0 1
refdes=R307
T 45500 61800 5 10 1 1 0 0 1
value=3.6k
T 45500 61800 5 10 1 1 0 0 1
footprint=0805
}
C 42900 64500 1 270 0 resistor-2.sym
{
T 43250 64100 5 10 0 0 270 0 1
device=RESISTOR
T 43200 64300 5 10 1 1 270 0 1
refdes=R308
T 42900 64500 5 10 1 1 0 0 1
value=0.33
T 42900 64500 5 10 0 1 0 0 1
footprint=0805
}
C 43600 61000 1 270 0 resistor-2.sym
{
T 43950 60600 5 10 0 0 270 0 1
device=RESISTOR
T 43900 60800 5 10 1 1 270 0 1
refdes=R309
T 43600 61000 5 10 0 0 0 0 1
footprint=0805
T 43600 61000 5 10 1 1 0 0 1
value=1.2k
}
C 41300 61000 1 270 0 capacitor-2.sym
{
T 42000 60800 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 41800 60800 5 10 1 1 270 0 1
refdes=C309
T 42200 60800 5 10 0 0 270 0 1
symversion=0.1
T 41300 61000 5 10 1 1 0 0 1
value=100u
T 41300 61000 5 10 0 0 0 0 1
footprint=TANT_D
}
C 46600 63400 1 270 0 capacitor-2.sym
{
T 47300 63200 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 47100 63200 5 10 1 1 270 0 1
refdes=C310
T 47500 63200 5 10 0 0 270 0 1
symversion=0.1
T 46600 63400 5 10 1 1 0 0 1
value=470p
T 46600 63400 5 10 0 0 0 0 1
footprint=0805
}
C 49500 61000 1 270 0 capacitor-2.sym
{
T 50200 60800 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 50000 60800 5 10 1 1 270 0 1
refdes=C311
T 50400 60800 5 10 0 0 270 0 1
symversion=0.1
T 49500 61000 5 10 1 1 0 0 1
value=100u
T 49500 61000 5 10 0 1 0 0 1
footprint=TANT_D
}
C 52300 61000 1 270 0 capacitor-2.sym
{
T 53000 60800 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 52800 60800 5 10 1 1 270 0 1
refdes=C312
T 53200 60800 5 10 0 0 270 0 1
symversion=0.1
T 52300 61000 5 10 1 1 0 0 1
value=100u
T 52300 61000 5 10 0 1 0 0 1
footprint=TANT_D
}
C 48900 63500 1 270 0 inductor-1.sym
{
T 49400 63300 5 10 0 0 270 0 1
device=INDUCTOR
T 49200 63300 5 10 1 1 270 0 1
refdes=L301
T 49600 63300 5 10 0 0 270 0 1
symversion=0.1
T 48900 63500 5 10 0 0 0 0 1
footprint=SRR6038
T 48900 63500 5 10 1 1 0 0 1
value=220u
}
C 50500 61600 1 0 0 inductor-1.sym
{
T 50700 62100 5 10 0 0 0 0 1
device=INDUCTOR
T 50700 61900 5 10 1 1 0 0 1
refdes=L302
T 50700 62300 5 10 0 0 0 0 1
symversion=0.1
T 50500 61600 5 10 0 0 0 0 1
footprint=SRR6038
T 50500 61600 5 10 1 1 0 0 1
value=1u
}
C 42300 61000 1 270 0 capacitor-1.sym
{
T 43000 60800 5 10 0 0 270 0 1
device=CAPACITOR
T 42800 60800 5 10 1 1 270 0 1
refdes=C313
T 43200 60800 5 10 0 0 270 0 1
symversion=0.1
T 42300 61000 5 10 0 0 0 0 1
footprint=0805
T 42300 61000 5 10 1 1 0 0 1
value=100n
}
C 53100 61000 1 270 0 capacitor-1.sym
{
T 53800 60800 5 10 0 0 270 0 1
device=CAPACITOR
T 53600 60800 5 10 1 1 270 0 1
refdes=C314
T 54000 60800 5 10 0 0 270 0 1
symversion=0.1
T 53100 61000 5 10 1 1 0 0 1
value=100n
T 53100 61000 5 10 0 1 0 0 1
footprint=0805
}
C 44300 57800 1 0 0 gnd-1.sym
N 46200 63100 46400 63100 4
N 46200 63400 46800 63400 4
C 48200 62600 1 90 0 zener-1.sym
{
T 47600 63000 5 10 0 0 90 0 1
device=ZENER_DIODE
T 47700 62900 5 10 1 1 90 0 1
refdes=Z301
T 48200 62600 5 10 1 1 0 0 1
comment=1N58192
T 48200 62600 5 10 0 1 0 0 1
footprint=DO214AC
}
N 44600 61700 43700 61700 4
N 43700 61000 43700 63100 4
N 44100 63100 43700 63100 4
C 46300 61900 1 0 0 gnd-1.sym
C 40000 63200 1 0 0 diode-1.sym
{
T 40400 63800 5 10 0 0 0 0 1
device=DIODE
T 40300 63700 5 10 1 1 0 0 1
refdes=D302
T 40000 63200 5 10 0 0 0 0 1
footprint=1206
}
C 55700 61600 1 0 0 output-2.sym
{
T 56600 61800 5 10 1 1 0 0 1
net=VCC5V:1
T 55900 62300 5 10 0 0 0 0 1
device=none
T 56600 61700 5 10 0 1 0 1 1
value=OUTPUT
}
C 41400 65000 1 0 0 output-2.sym
{
T 42300 65200 5 10 1 1 0 0 1
net=VBUS_D:1
T 41600 65700 5 10 0 0 0 0 1
device=none
T 42300 65100 5 10 0 1 0 1 1
value=OUTPUT
}
C 38100 63300 1 0 0 input-2.sym
{
T 38100 63500 5 10 1 1 0 0 1
net=VBUS:1
T 38700 64000 5 10 0 0 0 0 1
device=none
T 38600 63400 5 10 0 1 0 7 1
value=INPUT
}
C 55100 61100 1 270 0 resistor-2.sym
{
T 55450 60700 5 10 0 0 270 0 1
device=RESISTOR
T 55400 60900 5 10 1 1 270 0 1
refdes=R310
T 55100 61100 5 10 1 1 0 0 1
value=4.7k
T 55100 61100 5 10 0 0 0 0 1
footprint=0805
}
C 55000 59900 1 270 0 led-1.sym
{
T 55600 59100 5 10 0 0 270 0 1
device=LED
T 55400 59100 5 10 1 1 270 0 1
refdes=LED302
T 55800 59100 5 10 0 0 270 0 1
symversion=0.1
T 55000 59900 5 10 0 0 0 0 1
footprint=1206
}
N 39500 63400 40000 63400 4
N 55200 60200 55200 59900 4
B 38000 57600 19500 8000 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
T 38200 57700 9 10 1 0 0 0 1
Power
N 44100 63400 40900 63400 4
N 41400 65100 41200 65100 4
N 41200 65100 41200 63400 4
N 43000 63600 43000 63400 4
N 44100 64000 43600 64000 4
N 43600 63700 43600 64900 4
N 43000 64500 43000 64900 4
N 43000 64900 46600 64900 4
N 44100 63700 43600 63700 4
N 41500 63400 41500 61000 4
N 41500 60100 41500 58400 4
N 41500 58400 55200 58400 4
N 46200 63700 49000 63700 4
N 48000 63700 48000 63500 4
N 44400 58100 44400 58400 4
N 43700 60100 43700 58400 4
N 49000 63700 49000 63500 4
N 49000 62600 49000 61700 4
N 45500 61700 50500 61700 4
N 49700 61700 49700 61000 4
N 49700 60100 49700 58400 4
N 46600 64000 46600 64900 4
N 46600 64000 46200 64000 4
N 51400 61700 55700 61700 4
N 52500 61700 52500 61000 4
N 53300 61700 53300 61000 4
N 55200 61700 55200 61100 4
N 55200 58400 55200 59000 4
N 52500 60100 52500 58400 4
N 53300 60100 53300 58400 4
N 46400 63100 46400 62200 4
C 46700 61900 1 0 0 gnd-1.sym
C 47900 61900 1 0 0 gnd-1.sym
N 46800 62500 46800 62200 4
N 48000 62600 48000 62200 4
N 42500 61000 42500 63400 4
N 42500 60100 42500 58400 4
C 34900 65400 1 270 0 terminal-1.sym
{
T 35650 65090 5 10 0 0 270 0 1
device=terminal
T 35500 65090 5 10 0 0 270 0 1
footprint=M4_plated
T 34950 65150 5 10 1 1 270 6 1
refdes=T301
}
C 34900 62900 1 270 0 terminal-1.sym
{
T 35650 62590 5 10 0 0 270 0 1
device=terminal
T 35500 62590 5 10 0 0 270 0 1
footprint=M4_plated
T 34950 62650 5 10 1 1 270 6 1
refdes=T302
}
C 35900 60700 1 0 0 gnd-1.sym
C 36200 63700 1 0 0 output-2.sym
{
T 37100 63900 5 10 1 1 0 0 1
net=VBUS:1
T 36400 64400 5 10 0 0 0 0 1
device=none
T 37100 63800 5 10 0 1 0 1 1
value=OUTPUT
}
N 35000 63800 36200 63800 4
C 35800 62900 1 270 0 capacitor-2.sym
{
T 36500 62700 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 36300 62700 5 10 1 1 270 0 1
refdes=C315
T 36700 62700 5 10 0 0 270 0 1
symversion=0.1
T 35800 62900 5 10 1 1 0 0 1
value=100u
T 35800 62900 5 10 0 0 0 0 1
footprint=CPolar300
}
N 35000 64500 35000 63800 4
N 36000 63800 36000 62900 4
N 36000 62000 36000 61000 4
N 35000 62000 35000 61500 4
N 35000 61500 36000 61500 4
