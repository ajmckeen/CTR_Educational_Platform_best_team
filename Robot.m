classdef Robot < handle

    properties
        % Save vector of tubes and size of vector
        tubes = []
        num_tubes = 0

        % Save link lengths, phi values, and kappa values vectors (1 x num
        % links)
        lls = []
        phi = []
        kappa = []
    end

    methods
        % Constructor. This creates an instance of the robot class 
        % Must pass in a vector of tubes
        function self = Robot(tubes)
            self.tubes = tubes;
            self.num_tubes = size(tubes, 2);
        end

        % Here we calculate the kinematics of a full CTR
        % Pass in the raw joint variables
        % Return transformation matrix that goes from home frame to end
        % effector frame
        % See functions below for each step
        function T = fkin(self, q_var)  
            % First we get the rho and theta avlues from q_var
            rho = get_rho_values(self, q_var);
            theta = get_theta(self, q_var);

            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta, rho);
            disp('aaaaaaaaaaaaaaaaaaaaaaa')
            self.phi
            rad2deg(self.phi(1))

            % Finally we calculate the base to end effector transform
            T = calculate_transform(self, self.lls, self.phi, self.kappa);
        end

        % Get rho values from joint positions
        % Return rho (1 x i vector, i is num tubes)
        function rho = get_rho_values(self, q_var)
            % Here we extract rho values from the q_var vector

            % Initialize a vector to hold the result
            rho = zeros([1 self.num_tubes]);

            for i=2:self.num_tubes
                rho(i) = (q_var(i) - q_var(1)) * 10^-3;
            end

        end

        % Function to find the link lengths, in order
        % Returns link lengths (1 x j vector, where j is num links)
        function s = get_links(self, rho)
            % d: Length of the curved section for each tube [m]
            % rho: Starting points of the curved sections for each tube [m]
            
            % Initialize an array to hold the transition points
            transitionPoints = zeros(1, 2 * self.num_tubes);
            
            % Calculate the transition points (start and end of curved section)
            for i = 1:self.num_tubes
                transitionPoints(2*i - 1) = rho(i);         % Start of curved section
                %transitionPoints(2*i) = rho(i) + d(i);      % End of curved section
                transitionPoints(2*i) = rho(i) + self.tubes(i).d;
            end
            
            % Sort the transition points in ascending order
            sortedTransitionPoints = sort(transitionPoints);
            
            % Calculate the link lengths (differences between consecutive transition points)
            s = diff(sortedTransitionPoints);  % Include base (0) as starting point
        end

        % Function to get theta values
        % Returns theta (1 x j vector where j is num links)
        function theta = get_theta(self, q_var)
            % Here we extract theta values from the q_var vector

            % Initialize a vector to hold the result
            theta = zeros([1 self.num_tubes]);

            for i=1:self.num_tubes
                theta(i) = deg2rad(q_var(i+self.num_tubes));
            end
        end

        function [chi,gamma] = linkcurvature(E, OD, ID, k, theta)
            % E: Young's Modulus of the material for each tube [N/m^2]
            % OD: Outer diameters of the tubes [m]
            % ID: Inner diameters of the tubes [m]
            % k: Precurvatures of the tubes [m^-1]
            % theta: Rotations of the tubes [rad]
            
        
            % Number of tubes
            nTubes = length(OD);
            
            % Initialize the second moment of inertia array
            I = zeros(1, nTubes);
            
            % Calculate the second moment of inertia for each tube
            for i = 1:nTubes
                I(i) = (pi/64) * (OD(i)^4 - ID(i)^4);
            end
            
            % Initialize sums in the numerator and denominator for chi and gamma
            sum_EIk_cosTheta = 0;
            sum_EIk_sinTheta = 0;
            sum_EI = 0;
            
            % Calculate the sums for chi and gamma
            for i = 1:nTubes
                sum_EIk_cosTheta = sum_EIk_cosTheta + E(i) * I(i) * k(i) * cos(theta(i));
                sum_EIk_sinTheta = sum_EIk_sinTheta + E(i) * I(i) * k(i) * sin(theta(i));
                sum_EI = sum_EI + E(i) * I(i);
            end
            
            % Calculate chi and gamma
            chi = sum_EIk_cosTheta / sum_EI;
            gamma = sum_EIk_sinTheta / sum_EI;
        end


        % Function to calcualte phi values for two or three tube
        % configurations
        % Should return phi (1 x j vector, where j is num links)
        % and K (1 x j vector)
        function [phi,K] = calculate_phi_and_kappa(self, theta, rho)
            % Pre-allocate arrays for the properties of each tube
            E = zeros(1, self.num_tubes);
            OD = zeros(1, self.num_tubes);
            ID = zeros(1, self.num_tubes);
            k = zeros(1, self.num_tubes);
            d = zeros(1, self.num_tubes);
            for tubeIdx = 1:self.num_tubes
                E(tubeIdx) = self.tubes(tubeIdx).E;
                OD(tubeIdx) = self.tubes(tubeIdx).od;
                ID(tubeIdx) = self.tubes(tubeIdx).id;
                k(tubeIdx) = self.tubes(tubeIdx).k;
                alpha = theta;
                rho = rho;
                d(tubeIdx) = self.tubes(tubeIdx).d;

            end
            


            function curvatureInputs = prepareLinkCurvatureInputs(E, OD, ID, k, theta, rho, d)
                % Determine which tubes are present (and at what curvatures) in all of the links
            
                % E: Young's Modulus of the material for each tube [N/m^2]
                % OD: Outer diameters of the tubes [m]
                % ID: Inner diameters of the tubes [m]
                % k: Precurvatures of the tubes [m^-1]
                % theta: Rotations of the tubes [rad]
                % rho: Starting points of the curved sections [m]
                % d: Length of the curved sections for each tube [m]
                
                % Number of tubes
                nTubes = length(rho);
                
                % Calculate the transition points (start and end of curved section)
                transitionPoints = zeros(1, 2 * nTubes);
                for i = 1:nTubes
                    transitionPoints(2*i - 1) = rho(i);         % Start of curved section
                    transitionPoints(2*i) = rho(i) + d(i);      % End of curved section
                end
                
                % Sort the transition points in ascending order
                sortedTransitionPoints = sort(transitionPoints);
                
                % Initialize a cell array to store all the inputs for each link
                curvatureInputs = cell(1, length(sortedTransitionPoints) - 1);
                
                % Loop through each link (between consecutive transition points)
                for linkIdx = 1:(length(sortedTransitionPoints) - 1)
                    % Determine the current link start and end
                    linkStart = sortedTransitionPoints(linkIdx);
                    linkEnd = sortedTransitionPoints(linkIdx + 1);
                    
                    % Initialize arrays for E, OD, ID, k, and theta for the active tubes in this link
                    activeE = [];
                    activeOD = [];
                    activeID = [];
                    activeK = [];
                    activeTheta = [];
                    
                    % Loop through each tube and check if it's active in this link
                    for tubeIdx = 1:nTubes
                        tubeStart = rho(tubeIdx);
                        tubeEnd = rho(tubeIdx) + d(tubeIdx);
                        
                        % A tube is active from the base of the robot (not just after rho)
                        if linkStart >= 0 && linkEnd <= tubeEnd
                            % Include the tube
                            activeE = [activeE, E(tubeIdx)];
                            activeOD = [activeOD, OD(tubeIdx)];
                            activeID = [activeID, ID(tubeIdx)];
                            activeTheta = [activeTheta, theta(tubeIdx)];
                            
                            % Assign pre-curvature: 0 if the link is before the tube's rho, actual value if after
                            if linkStart < tubeStart
                                activeK = [activeK, 0];  % No pre-curvature before rho
                            else
                                activeK = [activeK, k(tubeIdx)];  % Actual pre-curvature after rho
                            end
                        end
                    end
                    
                    % Store the active tube information for this link
                    curvatureInputs{linkIdx} = struct('E', activeE, 'OD', activeOD, 'ID', activeID, ...
                                                      'k', activeK, 'theta', activeTheta);
                end
            end

            function [chi,gamma] = linkcurvature(E, OD, ID, k, theta)
                % E: Young's Modulus of the material for each tube [N/m^2]
                % OD: Outer diameters of the tubes [m]
                % ID: Inner diameters of the tubes [m]
                % k: Precurvatures of the tubes [m^-1]
                % theta: Rotations of the tubes [rad]
                
                % Number of tubes
                nTubes = length(OD);
                
                % Initialize the second moment of inertia array
                I = zeros(1, nTubes);
                
                % Calculate the second moment of inertia for each tube
                for i = 1:nTubes
                    I(i) = (pi/64) * (OD(i)^4 - ID(i)^4);
                end
                
                % Initialize sums in the numerator and denominator for chi and gamma
                sum_EIk_cosTheta = 0;
                sum_EIk_sinTheta = 0;
                sum_EI = 0;
                
                % Calculate the sums for chi and gamma
                for i = 1:nTubes
                    sum_EIk_cosTheta = sum_EIk_cosTheta + E(i) * I(i) * k(i) * cos(theta(i));
                    sum_EIk_sinTheta = sum_EIk_sinTheta + E(i) * I(i) * k(i) * sin(theta(i));
                    sum_EI = sum_EI + E(i) * I(i);
                end
                
                % Calculate chi and gamma
                chi = sum_EIk_cosTheta / sum_EI;
                gamma = sum_EIk_sinTheta / sum_EI;
            end

            curvatureInputs = prepareLinkCurvatureInputs(E, OD, ID, k, alpha, rho, d);
            num_links = length(curvatureInputs);
            % Initialize arrays to store chi and gamma for each link
            chi_all = zeros(1, num_links);
            gamma_all = zeros(1, num_links);

            % Loop through each link and calculate chi and gamma
            for linkIdx = 1:num_links
                % Get the curvature inputs for the current link
                linkData = curvatureInputs{linkIdx};
                
                % Extract the relevant parameters for the link
                E_link = linkData.E;
                OD_link = linkData.OD;
                ID_link = linkData.ID;
                k_link = linkData.k;
                theta_link = linkData.theta;
                
                % Call linkcurvature to calculate chi and gamma for the current link
                [chi, gamma] = linkcurvature(E_link, OD_link, ID_link, k_link, theta_link);
                
                % Store the results
                chi_all(linkIdx) = chi;
                gamma_all(linkIdx) = gamma;
            end

            % Initialize arrays to store phi and K for each link
            phi = zeros(1, size(self.lls,2));
            K = zeros(1, size(self.lls,2));

            % Loop through each link and calculate phi and K
            phi_sum = 0;
            for j = 1:num_links
                % Calculate the link rotation phi_j
                next_phi = atan2(gamma_all(j), chi_all(j));
                phi(j) = next_phi - phi_sum;
                phi_sum = phi_sum + phi(j);
                
                % Calculate the link curvature kappa_j
                K(j) = sqrt(chi_all(j)^2 + gamma_all(j)^2);
            end
            
        end

        % Take in all robot dependent parameters (lls, phi, kappa) and
        % compelte the robot independent constant curvature kinamtatics
        % Returns a 4x4 transformation matrix from base frame to end
        % effector
        function T = calculate_transform(self, s, phi, K)
            % Initialize T as the identity matrix (4x4)
            T = eye(4);
            
            % Loop through each link
            for i = 1:length(s)
                % Get the i-th link parameters
                l_i = s(i);   % Link length
                phi_i = phi(i);  % Orientation angle
                k_i = K(i);    % Curvature

                % Calculate the transformation matrix for the i-th link
                if k_i == 0
                    Ti_0 = [
                    cos(phi_i)*cos(k_i*l_i),  -sin(phi_i),   cos(phi_i)*sin(k_i*l_i),  0;
                    sin(phi_i)*cos(k_i*l_i),  cos(phi_i),    sin(phi_i)*sin(k_i*l_i),  0;
                    -sin(k_i*l_i),            0,            cos(k_i*l_i),              l_i;
                    0,                        0,            0,                         1
                    ];
                else
                    Ti_0 = [
                    cos(phi_i)*cos(k_i*l_i),  -sin(phi_i),   cos(phi_i)*sin(k_i*l_i),  (cos(phi_i)*(1 - cos(k_i*l_i)))/k_i;
                    sin(phi_i)*cos(k_i*l_i),  cos(phi_i),    sin(phi_i)*sin(k_i*l_i),  (sin(phi_i)*(1 - cos(k_i*l_i)))/k_i;
                    -sin(k_i*l_i),            0,            cos(k_i*l_i),              sin(k_i*l_i)/k_i;
                    0,                        0,            0,                         1
                    ];
                end 

                
                % Multiply the accumulated transformation matrix by the current one
                T = T * Ti_0;
            end
        end
    end
end

