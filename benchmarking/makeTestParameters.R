# Generate test parameters for benchmarking stimjim 
# (c) Nathan Cermak 2019
#
#

projectDirectory = 'C:/Users/Jackie/Desktop/cermak/stimjim/'
projectDirectory = '/home/x/Desktop/stimjim/'

duration = 1e6
paramMat = NULL
mode0 = 0
mode1 = 1
amp = c(10,1)
periods = round(exp(seq(log(250), log(4e5), length.out=30)))
pulseDurations = round(exp(seq(log(25),log(5000), length.out=30)))
ampScalings = seq(20,1000,length.out=30)

ampScaling=500; pulseDuration=100
for (period in periods){
  params = c(mode0, mode1, period, duration, round(amp*ampScaling), pulseDuration, -round(amp*ampScaling), pulseDuration)
  paramMat = rbind(paramMat, params)        
}

ampScaling=500; period=10000;
for (pulseDuration in pulseDurations){
  params = c(mode0, mode1, period, duration, round(amp*ampScaling), pulseDuration, -round(amp*ampScaling), pulseDuration)
  paramMat = rbind(paramMat, params)        
}

pulseDuration=100; period=1000;
for (ampScaling in ampScalings){
  params = c(mode0, mode1, period, duration, round(amp*ampScaling), pulseDuration, -round(amp*ampScaling), pulseDuration)
  paramMat = rbind(paramMat, params)        
}

par2string = function(p){
  sprintf("S0,%d,%d,%d,%d;%d,%d,%d;%d,%d,%d;\n", p[1], p[2], p[3],p[4], p[5], p[6],p[7],p[8],p[9],p[10])
}

stringVector = apply(paramMat,1,par2string)
cat(stringVector, file=paste(projectDirectory,"benchmarking/paramStringFile.txt", sep=''), sep='')
write.csv(paramMat, paste(projectDirectory,"benchmarking/paramMat.csv", sep=''),row.names = FALSE)


# ###### testing with serial connection
# con <- serial::serialConnection(name = "testcon",port = "ttyACM0",
#                         mode = "256000,n,8,1",
#                         newline = 0,
#                         translation = "auto")
# open(con)
# serial::read.serialConnection(con)
# 
# # write some stuff
# for (i in 1:10){
#   serial::write.serialConnection(con, par2string(paramMat[i,]))
#   serial::write.serialConnection(con, "T0\n")
#   print(serial::read.serialConnection(con))
#   Sys.sleep(1.5)
#   # read, in case something came in
#   print(serial::read.serialConnection(con))
# }
# # close the connection
# close(con)
