---
layout: post
title: "Unitree Frequency Response Characterization"
date: 2026-06-09 00:00:00 -0000
categories: technical
---

This document summarizes experimental frequency-response measurements of the Unitree locomotion stack using only:

* Command input: `/cmd_vel`
* State estimation: `/insight/vio_100hz`

The goal is to estimate the dynamic characteristics of:

* Forward velocity channel (`vx`)
* Yaw-rate channel (`vyaw`)

and derive practical controller design guidelines.

## Test Method

### Forward Velocity Sweep

Command:

$$
v_x(t)=v_0+A\sin(2\pi f t)
$$

where:

```text
v0 = 0.25 m/s
A  = 0.20 m/s
```

Resulting command range:

```text
0.05 ~ 0.45 m/s
```

The positive offset avoids reverse motion, which was found to introduce yaw coupling and corrupt low-frequency measurements.

### Yaw Rate Sweep

Command:

$$
\omega_z(t)=A\sin(2\pi f t)
$$

Measured yaw is extracted from the VIO orientation using:

```python
forward = rotation @ [0, 0, 1]
yaw = atan2(forward_y, forward_x)
```

Yaw is unwrapped and fitted with:

$$
\theta(t)
=
a\sin(\omega t)
+
b\cos(\omega t)
+
c
+
dt
$$

where:

$$
\omega = 2\pi f
$$

The measured yaw-rate amplitude is:

$$
\omega_{meas}
=
\omega
\sqrt{a^2+b^2}
$$

Frequency-response gain:

$$
Gain
=
\frac{\omega_{meas}}
{\omega_{cmd}}
$$

Phase lag:

$$
Lag
=
-\left(
\phi_{meas}
-
\phi_{cmd}
\right)
$$

Delay estimate:

$$
Delay
=
\frac{Lag}
{2\pi f}
$$

## Static Velocity Calibration

Constant command:

```text
vx = 0.20 m/s
```

Expected displacement:

```text
1.0 m
```

Measured:

```text
fit_vx = 0.127 m/s
translation = 0.625 m
```

Static gain:

$$
K_{vx}
=
0.127/0.20
=
0.635
$$

The commanded velocity does not map directly to physical velocity.

## Forward Velocity Response

### Measured Frequency Response

| Frequency (Hz) | Gain | Phase (rad) | Delay (s) |
| ------------- | ---- | ----------- | --------- |
| 0.5 | 1.044 | -0.449 | 0.143 |
| 1.0 | 0.939 | -0.708 | 0.113 |
| 2.0 | 0.778 | -1.539 | 0.122 |
| 3.0 | 0.461 | -2.551 | 0.135 |
| 4.0 | 0.372 | -2.844 | 0.113 |

### Bandwidth Estimate

The -3 dB point corresponds to:

$$
Gain = 0.707
$$

Observed:

```text
2 Hz -> 0.778
3 Hz -> 0.461
```

Estimated bandwidth:

$$
f_{bw,vx}
\approx
2.2Hz
$$

### Delay Estimate

Observed delay:

```text
0.11 ~ 0.14 s
```

Approximate delay:

$$
L_{vx}
\approx
0.13s
$$

### Identified Model

A first-order-plus-delay model fits the data reasonably well:

$$
G_{vx}(s)
\approx
\frac{e^{-0.13s}}
{0.07s+1}
$$

where:

$$
\tau
\approx
\frac{1}{2\pi\cdot2.2}
\approx
0.07s
$$

## Yaw Rate Response

### Sweep Amplitude Study

At low amplitudes, yaw response exhibits strong nonlinearity.

| Amplitude (rad/s) | Gain @ 0.5 Hz |
| ----------------: | ------------: |
|              0.10 |         0.164 |
|              0.20 |         0.222 |
|              0.30 |         0.309 |
|              0.50 |         0.515 |
|              1.00 |         0.811 |
|              1.50 |         0.898 |

This indicates:

* dead zone
* gait nonlinearities
* poor small-signal response

Frequency-response measurements are therefore performed using:

```text
Amplitude = 1.0 rad/s
```

or larger.

### Measured Frequency Response

| Frequency (Hz) |  Gain | Phase (rad) | Delay (s) |
| -------------: | ----: | ----------: | --------: |
|            0.5 | 0.810 |      -0.118 |     0.038 |
|            1.0 | 0.823 |      -0.314 |     0.050 |
|            2.0 | 0.872 |      -0.805 |     0.064 |
|            4.0 | 0.974 |      -1.661 |     0.066 |
|            6.0 | 0.804 |      -2.997 |     0.080 |

### Delay Estimate

Observed delay:

```text
0.04 ~ 0.08 s
```

Approximate delay:

$$
L_{yaw}
\approx
0.06s
$$

### Bandwidth Estimate

Unlike the forward velocity channel, gain remains high throughout the tested range:

```text
0.5 Hz -> 0.81
1.0 Hz -> 0.82
2.0 Hz -> 0.87
4.0 Hz -> 0.97
6.0 Hz -> 0.80
```

No clear -3 dB point was observed.

Therefore:

$$
f_{bw,yaw}
>
6Hz
$$

within the tested range.

### Identified Model

The yaw-rate channel behaves approximately as:

$$
G_{yaw}(s)
\approx
0.85e^{-0.06s}
$$

within the tested operating range.

This should be interpreted as an empirical model rather than a full system identification result.

## Comparison

| Channel | Bandwidth |   Delay | Approximate Model   |
| ------- | --------: | ------: | ------------------- |
| vx      |   ~2.2 Hz | ~0.13 s | First-order + delay |
| vyaw    |     >6 Hz | ~0.06 s | Gain + delay        |

## Control Design Implications

### Forward Position Control

Since:

$$
f_{bw,vx}
\approx
2.2Hz
$$

Recommended outer-loop bandwidth:

```text
0.5 ~ 0.8 Hz
```

Upper limit:

```text
~1 Hz
```

Higher values are likely to produce:

* oscillation
* overshoot
* path tracking instability

### Heading Control

Since:

$$
f_{bw,yaw}
>
6Hz
$$

Heading control can generally be designed much more aggressively than translational control.

In practice:

```text
yaw loop bandwidth
>>
position loop bandwidth
```

which matches observed locomotion behavior.

## Conclusions

Using only `/cmd_vel` and `/insight/vio_100hz`, it is possible to characterize the Unitree locomotion dynamics.

Key findings:

* Forward velocity bandwidth ≈ 2.2 Hz
* Forward velocity delay ≈ 130 ms
* Yaw-rate bandwidth > 6 Hz
* Yaw-rate delay ≈ 60 ms
* Yaw response is significantly faster than translational response
* Small-signal yaw commands exhibit strong nonlinear behavior
* A first-order-plus-delay model is adequate for `vx`
* A gain-plus-delay model is adequate for `vyaw` within the tested range

These measurements provide a useful basis for navigation controller tuning, trajectory tracking, and future MPC design.

<script>
  window.MathJax = {
    tex: {
      inlineMath: [['$', '$'], ['\\(', '\\)']],
      displayMath: [['$$', '$$'], ['\\[', '\\]']]
    }
  };
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
