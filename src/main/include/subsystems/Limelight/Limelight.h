#pragma once

namespace abval{
    class Limelight {  
    private:
        static Limelight* instance;

        //

        Limelight();
        ~Limelight();
    public:
        static Limelight* GetInstance();

        void setupPortForwarding();
    };
}  // namespace abval