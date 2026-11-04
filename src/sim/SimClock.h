#pragma once

#include <chrono>
#include <cmath>

// Fixed-timestep accumulator: physics always advances in kFixedDt-sized
// steps, decoupled from render frame time. main.cpp's loop calls Tick()
// once per frame, then drains ShouldStep()/ConsumeStep() in a catch-up
// loop before rendering once.
//
// Time scale changes how MANY steps run per real second, never their
// size. Every control loop and the flight dynamics integrator must sample
// at the same fixed rate -- JSBSim always advances by its own 1/120 s,
// so a scaled step would desync the aircraft from everything else, and a
// larger step would drop the effective control-loop frequency.
class SimClock {
public:
    static constexpr double kFixedDt = 1.0 / 120.0;

    // Sim seconds per real second. 0 pauses. Read freely; change it
    // through the methods below so a resume knows where to return to.
    double timeScale = 1.0;

    SimClock() : lastTime_(Clock::now()) {}

    bool Paused() const { return timeScale == 0.0; }

    void Pause() {
        if (Paused()) return;
        resumeScale_ = timeScale;
        timeScale = 0.0;
    }
    void Resume() { timeScale = resumeScale_; }
    void TogglePause() { Paused() ? Resume() : Pause(); }

    // Runs at `scale` from now on, and makes it what a later Resume
    // returns to.
    // A negative or non-finite scale is ignored: it would run the
    // accumulator backwards and the clock would never step again.
    void SetScale(double scale) {
        if (!(scale >= 0.0) || !std::isfinite(scale)) return;
        timeScale = scale;
        if (scale != 0.0) resumeScale_ = scale;
    }

    // The speed a run goes at: the live scale, or while paused the one
    // Resume returns to.
    double RunScale() const { return Paused() ? resumeScale_ : timeScale; }

    // Sets the run speed without pausing or resuming: while paused, only
    // what Resume returns to. 0 is ignored (that is Pause).
    void SetRunScale(double scale) {
        if (scale == 0.0 || !(scale > 0.0) || !std::isfinite(scale)) return;
        if (Paused()) {
            resumeScale_ = scale;
        } else {
            SetScale(scale);
        }
    }

    // Paused, with the first Resume going to `resumeScale` -- or to 1x if
    // that is not a positive number, so unpausing always visibly does something.
    void StartPaused(double resumeScale) {
        timeScale = 0.0;
        resumeScale_ = resumeScale > 0.0 && std::isfinite(resumeScale) ? resumeScale : 1.0;
    }

    void Tick() {
        Clock::time_point now = Clock::now();
        double frameTime = std::chrono::duration<double>(now - lastTime_).count();
        lastTime_ = now;
        Advance(frameTime);
    }

    // Tick with a given real frame time, in seconds (the part of Tick that
    // needs no wall clock).
    void Advance(double frameTime) {
        // Clamp so a debugger pause or window-drag stall doesn't queue up
        // hours of catch-up steps on the next Tick (the "spiral of death").
        constexpr double kMaxFrameTime = 0.25;
        if (frameTime > kMaxFrameTime) frameTime = kMaxFrameTime;

        accumulator_ += frameTime * timeScale;
    }

    bool ShouldStep() const { return accumulator_ >= kFixedDt; }

    // Gives up the steps still owed. For when stepping cannot keep up with
    // the requested speed: the run goes slower than asked instead of the
    // frame (and the window) waiting on the backlog.
    void DropBacklog() { accumulator_ = 0.0; }

    // Consumes one fixed step from the accumulator and returns its dt.
    double ConsumeStep() {
        accumulator_ -= kFixedDt;
        return kFixedDt;
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point lastTime_;
    double accumulator_ = 0.0;
    double resumeScale_ = 1.0;
};
