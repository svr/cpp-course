#include "common.hpp"
#include "coord.hpp"
#include "target_provider.hpp"

class JsonTargetProvider : public ITargetProvider {
    int targetCount{0};
    int timeSteps{0};
    Coord** targets;

    public:
    JsonTargetProvider() = default;
    ~JsonTargetProvider() override;
    void load(const char* file) override;
    int getTargetCount() const override;
    int getTargetTimeSteps() const override;
    Coord getTarget(int num, int timeIndex) const override;
};
