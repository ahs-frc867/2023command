#pragma once

namespace abval{
    class limelightNetworking {  
    private:
        static limelightNetworking* instance;

        //

        limelightNetworking();
        ~limelightNetworking();
    public:
        static limelightNetworking* GetInstance();

        void setupPortForwarding();
    };
}  // namespace abval