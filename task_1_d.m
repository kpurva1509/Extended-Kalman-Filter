close all;
class_1= [0, 3, 6];
class_2= [1, 2, 4, 5, 7, 8, 9];

num_locations = size(class_1, 2) + size(class_2, 2);

landmark_numbers = class_1;

p_landmark_at_class_1 = 0.8;
np_landmark_at_class_1 = 0.2;

p_landmark_at_class_2 = 0.4;
np_landmark_at_class_2 = 0.6;

control_input = [0, 3, 4];
observations = [1, 1, 0];

belief = ones(1, num_locations)*1/num_locations;

num_plots = length(control_input)*2;

for index1 = 1:length(control_input)
    control = control_input(index1);
    observe = observations(index1);
    
    belief_bar = zeros(1, num_locations);
    for index2 = 1:length(belief)
        belief_bar(mod((index2-1) + control, num_locations)+1) = belief(index2);
    end
    pl = subplot(num_plots, 1, 2*(index1-1)+1);
    bar(pl, 0:num_locations-1, belief_bar, 'stacked');
    for index2 = 1:length(belief)
        if observe
            if ismember(index2-1, landmark_numbers)
                belief_bar(index2) = p_landmark_at_class_1*belief_bar(index2);
            else
                belief_bar(index2) = p_landmark_at_class_2*belief_bar(index2);
            end
        else
            if ismember(index2-1, landmark_numbers)
                belief_bar(index2) = np_landmark_at_class_1*belief_bar(index2);
            else
                belief_bar(index2) = np_landmark_at_class_2*belief_bar(index2);
            end
        end
    end
    belief = belief_bar/sum(belief_bar);
    pl = subplot(num_plots, 1, 2*(index1-1)+2);
    bar(pl, 0:num_locations-1, belief, 'stacked');
end

figure;
bar(0:num_locations-1, belief);

