v 20110115 2
C 40000 40000 0 0 0 title-B.sym
C 41500 42900 1 0 0 mega48-tqfp32.sym
{
T 46000 49400 5 10 1 1 0 6 1
refdes=U?
T 41800 49700 5 10 0 0 0 0 1
device=ATMega48-TQFP32
T 41800 49900 5 10 0 0 0 0 1
footprint=TQFP32_7
T 41100 49700 5 10 1 0 0 0 1
comment=ATMega48/88/168
}
C 54500 47400 1 0 0 header3-1.sym
{
T 55500 48050 5 10 0 0 0 0 1
device=HEADER3
T 54900 48700 5 10 1 1 0 0 1
refdes=J?
}
C 54500 45500 1 0 0 header3-1.sym
{
T 55500 46150 5 10 0 0 0 0 1
device=HEADER3
T 54900 46800 5 10 1 1 0 0 1
refdes=J?
}
C 54500 43500 1 0 0 header3-1.sym
{
T 55500 44150 5 10 0 0 0 0 1
device=HEADER3
T 54900 44800 5 10 1 1 0 0 1
refdes=J?
}
C 32800 51400 1 0 0 input-2.sym
{
T 32800 51600 5 10 1 1 0 0 1
net=VCC5V:1
T 33400 52100 5 10 0 0 0 0 1
device=none
T 33300 51500 5 10 0 1 0 7 1
value=INPUT
}
C 43600 42000 1 0 0 gnd-1.sym
C 47300 37000 1 0 0 gnd-1.sym
C 47300 42600 1 270 0 input-2.sym
{
T 47500 42600 5 10 1 1 270 0 1
net=VCC5V:1
T 48000 42000 5 10 0 0 270 0 1
device=none
T 47400 42100 5 10 0 1 270 7 1
value=INPUT
}
N 44100 42600 44100 42900 4
N 34200 51500 48300 51500 4
N 43700 51500 43700 49600 4
N 44100 51500 44100 49600 4
C 47300 40800 1 270 0 resistor-2.sym
{
T 47650 40400 5 10 0 0 270 0 1
device=RESISTOR
T 47600 40600 5 10 1 1 270 0 1
refdes=R?
T 47300 40800 5 10 1 1 0 0 1
value=10k
}
C 47200 38400 1 270 0 capacitor-1.sym
{
T 47900 38200 5 10 0 0 270 0 1
device=CAPACITOR
T 47700 38200 5 10 1 1 270 0 1
refdes=C?
T 48100 38200 5 10 0 0 270 0 1
symversion=0.1
T 47200 38400 5 10 1 1 0 0 1
value=100u
}
C 46400 38400 1 270 0 switch-spst-1.sym
{
T 47100 38000 5 10 0 0 270 0 1
device=SPST
T 46700 38100 5 10 1 1 270 0 1
refdes=S?
}
N 47400 41200 47400 40800 4
N 47400 39900 47400 38400 4
N 47400 37500 47400 37300 4
N 47400 38700 46400 38700 4
N 46300 43700 46500 43700 4
C 38900 46800 1 270 0 crystal-1.sym
{
T 39400 46600 5 10 0 0 270 0 1
device=CRYSTAL
T 39200 46600 5 10 1 1 270 0 1
refdes=U?
T 39600 46600 5 10 0 0 270 0 1
symversion=0.1
T 38900 46800 5 10 1 1 0 0 1
value=20MHz
}
C 37900 46800 1 0 0 capacitor-1.sym
{
T 38100 47500 5 10 0 0 0 0 1
device=CAPACITOR
T 38100 47300 5 10 1 1 0 0 1
refdes=C?
T 38100 47700 5 10 0 0 0 0 1
symversion=0.1
T 37900 46800 5 10 1 1 0 0 1
value=22p
}
C 37900 45700 1 0 0 capacitor-1.sym
{
T 38100 46400 5 10 0 0 0 0 1
device=CAPACITOR
T 38100 46200 5 10 1 1 0 0 1
refdes=C?
T 38100 46600 5 10 0 0 0 0 1
symversion=0.1
T 37900 45700 5 10 1 1 0 0 1
value=22p
}
C 37500 45300 1 0 0 gnd-1.sym
N 38800 47000 41500 47000 4
N 39000 47000 39000 46800 4
N 39000 46100 39000 45900 4
N 38800 45900 39800 45900 4
N 37900 47000 37600 47000 4
N 37600 47000 37600 45600 4
N 37900 45900 37600 45900 4
N 41500 46700 39800 46700 4
N 39800 46700 39800 45900 4
C 49200 46400 1 0 0 gnd-1.sym
C 49100 47900 1 270 0 capacitor-1.sym
{
T 49800 47700 5 10 0 0 270 0 1
device=CAPACITOR
T 49600 47700 5 10 1 1 270 0 1
refdes=C?
T 50000 47700 5 10 0 0 270 0 1
symversion=0.1
T 49100 47900 5 10 1 1 0 0 1
value=100n
}
C 48200 49900 1 270 0 inductor-1.sym
{
T 48700 49700 5 10 0 0 270 0 1
device=INDUCTOR
T 48500 49700 5 10 1 1 270 0 1
refdes=L?
T 48900 49700 5 10 0 0 270 0 1
symversion=0.1
T 48200 49900 5 10 1 1 0 0 1
value=120u
}
C 48100 47900 1 270 0 capacitor-1.sym
{
T 48800 47700 5 10 0 0 270 0 1
device=CAPACITOR
T 48600 47700 5 10 1 1 270 0 1
refdes=C?
T 49000 47700 5 10 0 0 270 0 1
symversion=0.1
T 48100 47900 5 10 1 1 0 0 1
value=100n
}
N 46300 48500 49300 48500 4
N 49300 46700 49300 47000 4
N 48300 47900 48300 49000 4
N 46300 48200 48300 48200 4
N 48300 49900 48300 51500 4
N 46300 47900 46500 47900 4
C 48200 46400 1 0 0 gnd-1.sym
C 46400 47600 1 0 0 gnd-1.sym
N 48300 46700 48300 47000 4
N 49300 48500 49300 47900 4
C 39400 35100 1 0 0 AVR_ISP6-1.sym
{
T 40500 36300 5 10 1 1 0 6 1
refdes=J?
T 39700 36550 5 10 0 0 0 0 1
device=HEADER6
T 39700 36750 5 10 0 0 0 0 1
footprint=CON_HDR-254P-3C-2R-6N__Molex_8624-Series
}
C 40800 35000 1 0 0 gnd-1.sym
N 40800 35400 40900 35400 4
N 40900 35400 40900 35300 4
C 46500 46900 1 0 0 io-1.sym
{
T 47400 47100 5 10 1 1 0 0 1
net=A6:1
T 46700 47500 5 10 0 0 0 0 1
device=none
T 47400 47000 5 10 0 1 0 1 1
value=IO
}
C 46500 46600 1 0 0 io-1.sym
{
T 47400 46800 5 10 1 1 0 0 1
net=A7:1
T 46700 47200 5 10 0 0 0 0 1
device=none
T 47400 46700 5 10 0 1 0 1 1
value=IO
}
C 46500 45400 1 0 0 io-1.sym
{
T 47400 45600 5 10 1 1 0 0 1
net=A0:1
T 46700 46000 5 10 0 0 0 0 1
device=none
T 47400 45500 5 10 0 1 0 1 1
value=IO
}
C 46500 45100 1 0 0 io-1.sym
{
T 47400 45300 5 10 1 1 0 0 1
net=A1:1
T 46700 45700 5 10 0 0 0 0 1
device=none
T 47400 45200 5 10 0 1 0 1 1
value=IO
}
C 46500 44800 1 0 0 io-1.sym
{
T 47400 45000 5 10 1 1 0 0 1
net=A2:1
T 46700 45400 5 10 0 0 0 0 1
device=none
T 47400 44900 5 10 0 1 0 1 1
value=IO
}
C 46500 44500 1 0 0 io-1.sym
{
T 47400 44700 5 10 1 1 0 0 1
net=A3:1
T 46700 45100 5 10 0 0 0 0 1
device=none
T 47400 44600 5 10 0 1 0 1 1
value=IO
}
C 46500 44200 1 0 0 io-1.sym
{
T 47400 44400 5 10 1 1 0 0 1
net=A4:1
T 46700 44800 5 10 0 0 0 0 1
device=none
T 47400 44300 5 10 0 1 0 1 1
value=IO
}
C 46500 43900 1 0 0 io-1.sym
{
T 47400 44100 5 10 1 1 0 0 1
net=A5:1
T 46700 44500 5 10 0 0 0 0 1
device=none
T 47400 44000 5 10 0 1 0 1 1
value=IO
}
C 46500 43600 1 0 0 io-1.sym
{
T 47400 43800 5 10 1 1 0 0 1
net=RST:1
T 46700 44200 5 10 0 0 0 0 1
device=none
T 47400 43700 5 10 0 1 0 1 1
value=IO
}
C 41300 48700 1 0 1 io-1.sym
{
T 40400 48900 5 10 1 1 0 6 1
net=B0:1
T 41100 49300 5 10 0 0 0 6 1
device=none
T 40400 48800 5 10 0 1 0 7 1
value=IO
}
C 41300 48400 1 0 1 io-1.sym
{
T 40400 48600 5 10 1 1 0 6 1
net=B1:1
T 41100 49000 5 10 0 0 0 6 1
device=none
T 40400 48500 5 10 0 1 0 7 1
value=IO
}
C 41300 48100 1 0 1 io-1.sym
{
T 40400 48300 5 10 1 1 0 6 1
net=B2:1
T 41100 48700 5 10 0 0 0 6 1
device=none
T 40400 48200 5 10 0 1 0 7 1
value=IO
}
C 41300 45700 1 0 1 io-1.sym
{
T 40400 45900 5 10 1 1 0 6 1
net=RXI:1
T 41100 46300 5 10 0 0 0 6 1
device=none
T 40400 45800 5 10 0 1 0 7 1
value=IO
}
C 41300 45400 1 0 1 io-1.sym
{
T 40400 45600 5 10 1 1 0 6 1
net=TXO:1
T 41100 46000 5 10 0 0 0 6 1
device=none
T 40400 45500 5 10 0 1 0 7 1
value=IO
}
C 41300 45100 1 0 1 io-1.sym
{
T 40400 45300 5 10 1 1 0 6 1
net=D2:1
T 41100 45700 5 10 0 0 0 6 1
device=none
T 40400 45200 5 10 0 1 0 7 1
value=IO
}
C 41300 44800 1 0 1 io-1.sym
{
T 40400 45000 5 10 1 1 0 6 1
net=D3:1
T 41100 45400 5 10 0 0 0 6 1
device=none
T 40400 44900 5 10 0 1 0 7 1
value=IO
}
C 41300 44500 1 0 1 io-1.sym
{
T 40400 44700 5 10 1 1 0 6 1
net=D4:1
T 41100 45100 5 10 0 0 0 6 1
device=none
T 40400 44600 5 10 0 1 0 7 1
value=IO
}
C 41300 44200 1 0 1 io-1.sym
{
T 40400 44400 5 10 1 1 0 6 1
net=D5:1
T 41100 44800 5 10 0 0 0 6 1
device=none
T 40400 44300 5 10 0 1 0 7 1
value=IO
}
C 41300 43900 1 0 1 io-1.sym
{
T 40400 44100 5 10 1 1 0 6 1
net=D6:1
T 41100 44500 5 10 0 0 0 6 1
device=none
T 40400 44000 5 10 0 1 0 7 1
value=IO
}
C 41300 43600 1 0 1 io-1.sym
{
T 40400 43800 5 10 1 1 0 6 1
net=D7:1
T 41100 44200 5 10 0 0 0 6 1
device=none
T 40400 43700 5 10 0 1 0 7 1
value=IO
}
N 46500 45500 46300 45500 4
N 46500 45200 46300 45200 4
N 46500 44900 46300 44900 4
N 46500 46700 46300 46700 4
N 46500 47000 46300 47000 4
N 41300 48800 41500 48800 4
N 41300 48500 41500 48500 4
N 41300 48200 41500 48200 4
N 41300 45800 41500 45800 4
N 41300 45500 41500 45500 4
N 41300 45200 41500 45200 4
N 41300 44900 41500 44900 4
N 41300 44600 41500 44600 4
N 41300 44300 41500 44300 4
N 41300 44000 41500 44000 4
N 41300 43700 41500 43700 4
N 46300 44600 46500 44600 4
N 46300 44300 46500 44300 4
N 46300 44000 46500 44000 4
C 44900 39300 1 0 1 io-1.sym
{
T 44000 39500 5 10 1 1 0 6 1
net=DTR:1
T 44700 39900 5 10 0 0 0 6 1
device=none
T 44000 39400 5 10 0 1 0 7 1
value=IO
}
C 46000 39200 1 0 1 capacitor-1.sym
{
T 45800 39900 5 10 0 0 0 6 1
device=CAPACITOR
T 45800 39700 5 10 1 1 0 6 1
refdes=C?
T 45800 40100 5 10 0 0 0 6 1
symversion=0.1
T 46000 39200 5 10 1 1 0 6 1
value=0.1u
}
C 47000 34300 1 0 1 connector6-1.sym
{
T 45200 36100 5 10 0 0 0 6 1
device=CONNECTOR_6
T 46900 36300 5 10 1 1 0 6 1
refdes=CONN?
T 47000 34400 5 15 1 1 0 0 6
comment=DTR
RXI
TXO
VCC
CTS
GND
}
C 45000 35900 1 0 1 io-1.sym
{
T 44100 36100 5 10 1 1 0 6 1
net=DTR:1
T 44800 36500 5 10 0 0 0 6 1
device=none
T 44100 36000 5 10 0 1 0 7 1
value=IO
}
N 45000 36000 45300 36000 4
C 44900 34200 1 0 0 gnd-1.sym
C 44600 34500 1 0 0 gnd-1.sym
N 45300 34500 45000 34500 4
N 45300 34800 44700 34800 4
C 43600 35000 1 0 0 input-2.sym
{
T 43600 35200 5 10 1 1 0 0 1
net=VCC5V:1
T 44200 35700 5 10 0 0 0 0 1
device=none
T 44100 35100 5 10 0 1 0 7 1
value=INPUT
}
N 45000 35100 45300 35100 4
C 45000 35600 1 0 1 io-1.sym
{
T 44100 35800 5 10 1 1 0 6 1
net=TXO:1
T 44800 36200 5 10 0 0 0 6 1
device=none
T 44100 35700 5 10 0 1 0 7 1
value=IO
}
C 45000 35300 1 0 1 io-1.sym
{
T 44100 35500 5 10 1 1 0 6 1
net=RXI:1
T 44800 35900 5 10 0 0 0 6 1
device=none
T 44100 35400 5 10 0 1 0 7 1
value=IO
}
N 45000 35700 45300 35700 4
N 45300 35400 45000 35400 4
C 41000 35600 1 0 0 io-1.sym
{
T 41900 35800 5 10 1 1 0 0 1
net=MOSI:1
T 41200 36200 5 10 0 0 0 0 1
device=none
T 41900 35700 5 10 0 1 0 1 1
value=IO
}
C 39200 35900 1 0 1 io-1.sym
{
T 38300 36100 5 10 1 1 0 6 1
net=MISO:1
T 39000 36500 5 10 0 0 0 6 1
device=none
T 38300 36000 5 10 0 1 0 7 1
value=IO
}
C 39200 35600 1 0 1 io-1.sym
{
T 38300 35800 5 10 1 1 0 6 1
net=SCK:1
T 39000 36200 5 10 0 0 0 6 1
device=none
T 38300 35700 5 10 0 1 0 7 1
value=IO
}
C 42400 35900 1 0 1 input-2.sym
{
T 42400 36100 5 10 1 1 0 6 1
net=VCC5V:1
T 41800 36600 5 10 0 0 0 6 1
device=none
T 41900 36000 5 10 0 1 0 1 1
value=INPUT
}
C 39200 35300 1 0 1 io-1.sym
{
T 38300 35500 5 10 1 1 0 6 1
net=RST:1
T 39000 35900 5 10 0 0 0 6 1
device=none
T 38300 35400 5 10 0 1 0 7 1
value=IO
}
N 39400 36000 39200 36000 4
N 39200 35700 39400 35700 4
N 39200 35400 39400 35400 4
N 40800 35700 41000 35700 4
N 40800 36000 41000 36000 4
C 41300 47500 1 0 1 io-1.sym
{
T 40400 47700 5 10 1 1 0 6 1
net=MISO:1
T 41100 48100 5 10 0 0 0 6 1
device=none
T 40400 47600 5 10 0 1 0 7 1
value=IO
}
C 41300 47200 1 0 1 io-1.sym
{
T 40400 47400 5 10 1 1 0 6 1
net=SCK:1
T 41100 47800 5 10 0 0 0 6 1
device=none
T 40400 47300 5 10 0 1 0 7 1
value=IO
}
C 41300 47800 1 0 1 io-1.sym
{
T 40400 48000 5 10 1 1 0 6 1
net=MOSI:1
T 41100 48400 5 10 0 0 0 6 1
device=none
T 40400 47900 5 10 0 1 0 7 1
value=IO
}
C 47700 39600 1 0 0 io-1.sym
{
T 48600 39800 5 10 1 1 0 0 1
net=RST:1
T 47900 40200 5 10 0 0 0 0 1
device=none
T 48600 39700 5 10 0 1 0 1 1
value=IO
}
N 44900 39400 45100 39400 4
N 46000 39400 47400 39400 4
N 46400 38700 46400 38400 4
N 47700 39700 47400 39700 4
N 46400 37600 46400 37400 4
N 46400 37400 47400 37400 4
N 44100 42600 43700 42600 4
N 43700 42300 43700 42900 4
N 41300 47900 41500 47900 4
N 41300 47600 41500 47600 4
N 41300 47300 41500 47300 4
