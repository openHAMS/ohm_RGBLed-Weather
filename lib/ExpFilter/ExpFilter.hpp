template<class T> class ExpFilter
{
public:
    ExpFilter(T inital, T weight):
        currentValue(inital),
        weight(weight)
    { }

    T Filter(T newRaw)
    {
        currentValue = (weight * newRaw + (100 - weight) * currentValue) / 100;
        return currentValue;
    }

    void SetWeight(T newWeight)
    {
        weight = newWeight;
    }

    T GetWeight() const { return weight; }

    T Current() const { return currentValue; }

    T SetValue(T forcedValue)
    {
        currentValue = forcedValue;
        return currentValue;
    }

private:
    T currentValue;
    T weight;
};

template<> class ExpFilter<float>
{
public:
    ExpFilter(float initial, float weight):
        currentValue(initial),
        weight(weight / 100.0)
    { }


    float Filter(float newRaw)
    {
        currentValue = weight * newRaw + (1.0 - weight) * currentValue;
        return currentValue;
    }

    void SetWeight(float newWeight)
    {
        weight = newWeight / 100.0;
    }

    float GetWeight() const { return weight * 100.0; }

    float Current() const { return currentValue; }

    float SetValue(float forcedValue)
    {
        currentValue = forcedValue;
        return currentValue;
    }

private:
    float currentValue;
    float weight;
};
