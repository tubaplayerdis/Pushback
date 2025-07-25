//
// Created by aaron on 7/25/2025.
//

#ifndef TITANSELECT_HPP
#define TITANSELECT_HPP

#include <functional>

#include "../liblvgl/lvgl.h"
#include <cstring>
#include <string>
#include <utility>

#define SELECTOR ts::selector::Get()
#define SELECTOR_HEIGHT 200
#define SELECTOR_WIDTH 475
#define SELECTOR_ROWS 3
#define SELECTOR_COLS 3
#define SELECTOR_X_OFFSET 0
#define SELECTOR_Y_OFFSET -18
#define SELECTOR_NO_AUTON_TEXT "No Auton"

#pragma region autons

/// <summary>
/// Defines an auton
/// </summary>
/// <param name="name">Name of the auton</param>
/// <param name="lambda">Code of the auton in brackets {}</param>
/// <returns>None</returns>
#define AUTON(name, lambda) \
static ts::auton name(#name, []()lambda); \

namespace ts
{
    struct auton;

    class registry
    {
    public:
        inline static std::vector<auton*> autons;
        inline static void Register(auton* routine)
        {
            autons.push_back(routine);
        }
    };

    struct auton
    {
        const char* AutonName;
        const std::function<void()> FunctionRef;
        auton(const char* name, std::function<void()> func) : AutonName(name), FunctionRef(std::move(func))
        {
            registry::Register(this);
        }
    };
}

#pragma endregion

namespace ts
{
    class selector
    {
        inline static selector* instance;

        const char* aSelectedAuton;

        selector() : aSelectedAuton(nullptr) {}

        public:

        void Focus();

        void RunSelectedAuton() const;

        void RunAuton(const char* name) const;

        static selector* Get();

        static void HandleButtonMatrix(lv_event_t * e);
    };

    inline void selector::Focus()
    {
        lv_obj_t * btnm = lv_buttonmatrix_create(lv_screen_active());
        static const char* btn_map[SELECTOR_ROWS * SELECTOR_COLS + SELECTOR_COLS] = {};

        //Normally this would go rows->cols, but button matrix likes to be difficult
        int gIndex = 0;
        std::vector<auton*> autons = registry::autons;
        for (int i = 0; i < SELECTOR_COLS; i++)
        {
            for (int j = 0; j < SELECTOR_ROWS; j++)
            {
                //Item will not be valid
                if (autons.size() > gIndex)
                {
                    if (autons[gIndex] == nullptr)
                    {
                        btn_map[gIndex] = "Bruh";
                    } else
                    {
                        btn_map[gIndex] = autons[gIndex]->AutonName;
                    }
                } else
                {
                    btn_map[gIndex] = SELECTOR_NO_AUTON_TEXT;
                }
                gIndex++;
            }
            btn_map[gIndex] = "\n";
            gIndex++;
        }

        lv_buttonmatrix_set_map(btnm, btn_map);

        lv_obj_set_size(btnm, SELECTOR_WIDTH, SELECTOR_HEIGHT);
        lv_obj_align(btnm, LV_ALIGN_CENTER, SELECTOR_X_OFFSET, SELECTOR_Y_OFFSET);

        lv_obj_add_event_cb(btnm, selector::HandleButtonMatrix, LV_EVENT_VALUE_CHANGED, 0);

        //Create RunButton
        //Create Bottom label clarifying which auton is selected.
    }

    inline void selector::RunSelectedAuton() const
    {
        RunAuton(aSelectedAuton);
        //TODO: Handle no selected auton
    }

    inline void selector::RunAuton(const char *name) const
    {
        for (auton* autonomous : registry::autons)
        {
            if (strcmp(autonomous->AutonName, name) == 0) autonomous->FunctionRef();
        }
        //TODO: Handle no matched auton
    }

    inline selector * selector::Get()
    {
        //Lifetime heap variables do not need to be freed.
        if (!instance) instance = new selector;
        return instance;
    }

    inline void selector::HandleButtonMatrix(lv_event_t *e)
    {
    }
}

#endif //TITANSELECT_HPP
