# Nvidia_Isaac_Essentials



# People Animation

I needed to run a simulation app with people animation (omni.anim.people) and found that I was not able to find the position of the animation characters. I tried several solutions mentioned on the Nvidia developer forums but found all of them only provide the initial pose but not the current pose of the character. I then inspected the ``character_behavior.py`` file in the ``omni.anim.people-0.2.4`` folder, and found that the on_update() function gets called periodically and one can access the updated current position of the character from here.
``print(self.character_name, Utils.get_character_pos(self.character))``
