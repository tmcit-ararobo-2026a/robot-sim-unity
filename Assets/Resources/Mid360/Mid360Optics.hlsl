#ifndef ROBOTSIM_MID360_OPTICS
#define ROBOTSIM_MID360_OPTICS

static const float TenPercentRangeMeters = 40.0;

float TransparentReturnProbability(float transmission, float backscatter, float reflectivity,
    float incidence, float distance, float referenceRange)
{
    // Keep the range, probability and hash in sync with Mid360Surface.
    if (transmission >= 1) return 0;
    if (backscatter <= 0)
    {
        float incidenceFourth = incidence * incidence * incidence * incidence;
        return (1 - transmission) * incidenceFourth * incidenceFourth;
    }

    float rangeRatio = (referenceRange > 0 ? referenceRange : TenPercentRangeMeters) / max(distance, 1.0);
    float signal = backscatter * min(reflectivity, 150) / 15 *
        incidence * incidence * rangeRatio * rangeRatio;
    return signal / (1 + signal);
}

bool AcceptReturn(float probability, float3 direction, float phase)
{
    uint hash = asuint(direction.x) ^ asuint(direction.y) * 0x9e3779b9u
        ^ asuint(direction.z) * 0x85ebca6bu ^ asuint(phase);
    hash = (hash ^ (hash >> 16)) * 0x7feb352du;
    hash = (hash ^ (hash >> 15)) * 0x846ca68bu;
    float sampleValue = ((hash ^ (hash >> 16)) & 0x00ffffffu) / 16777216.0;
    return sampleValue < probability;
}

#endif
