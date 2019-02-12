# Generate test parameters for benchmarking microstim 
# (c) Nathan Cermak 2019
#
#

projectDirectory = '/home/x/Desktop/microstim/'

duration = 1e6
paramMat = NULL
mode0 = 0
mode1 = 2
amp = c(1000,100,100,100)[c(mode0,mode1)+1]
ampScalings = c(1,5)
pulseDurations = c(20,50,100,200,500,1000,2000)
periods = round(exp(seq(log(500), log(5e5), length.out=10)))

for (period in periods){
  for (pulseDuration in pulseDurations[pulseDurations<period/2]){
    for (ampScaling in ampScalings){
      params = c(mode0, mode1, period, duration, amp*ampScaling, pulseDuration, -amp*ampScaling, pulseDuration)
      paramMat = rbind(paramMat, params)        
    }
  }
}

par2string = function(p){
  sprintf("S0,%d,%d,%d,%d;%d,%d,%d;%d,%d,%d;\n", p[1], p[2], p[3],p[4], p[5], p[6],p[7],p[8],p[9],p[10])
}

stringVector = apply(paramMat,1,par2string)
cat(stringVector, file=paste(projectDirectory,"benchmarking/paramStringFile.txt", sep=''))
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
