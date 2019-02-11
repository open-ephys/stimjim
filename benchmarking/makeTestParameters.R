
duration = 1e6

paramMat = NULL

for (mode1 in 0:3){
  for(mode2 in 0:3){
    for (period in round(exp(seq(log(500), log(5e5), length.out=20)))){
      amp = c(1000,100,100,100)[c(mode1,mode2)+1]
      for (pulseDuration in c(20,30,40,50,75,100,200)){
        params = c(mode1, mode2, period, duration, amp, pulseDuration, -amp, pulseDuration)
        paramMat = rbind(paramMat, params)        
      }
      amp = amp*5
      for (pulseDuration in c(20,30,40,50,75,100,200)){
        params = c(mode1, mode2, period, duration, amp, pulseDuration, -amp, pulseDuration)
        paramMat = rbind(paramMat, params)        
      }
    }
  }
}

par2string = function(p){
  sprintf("S0,%d,%d,%d,%d;%d,%d,%d;%d,%d,%d;\n", p[1], p[2], p[3],p[4], p[5], p[6],p[7],p[8],p[9],p[10])
}

stringVector = apply(paramMat,1,par2string)
cat(stringVector, file="/home/x/Desktop/paramStringFile.txt")
write.csv(paramMat, "/home/x/Desktop/paramMat.csv")

con <- serial::serialConnection(name = "testcon",port = "ttyACM0",
                        mode = "256000,n,8,1",
                        newline = 0,
                        translation = "auto")
open(con)
serial::read.serialConnection(con)

# write some stuff
for (i in 1:10){
  serial::write.serialConnection(con, par2string(paramMat[i,]))
  serial::write.serialConnection(con, "T0\n")
  print(serial::read.serialConnection(con))
  Sys.sleep(1.5)
  # read, in case something came in
  print(serial::read.serialConnection(con))
}
# close the connection
close(con)
