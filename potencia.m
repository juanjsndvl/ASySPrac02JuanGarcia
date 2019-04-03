function [ ] = potencia( X,T )
P=(integral(X,-T/2,T/2))./T
rms=sqrt(P)
end
