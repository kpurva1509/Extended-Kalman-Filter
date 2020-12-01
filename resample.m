function [newSamples, newWeight] = resample(samples, weight)

numSamples = length(samples);

indices = randsample(1:numSamples, numSamples, true, weight);

newSamples = samples(:, indices);
newWeight = weight(indices);
newWeight = newWeight/sum(newWeight);

