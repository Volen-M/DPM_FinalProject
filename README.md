# McGill DPM Group 11 Version Control

## Commit Message Example
	* "02.02.01-Nav, Odo-Changed all functions to take in degrees"

## Project Software Milestones (First Number of Version Control)
	* 00 - Shell complete
	* 01 - Gear system functional
	* 02 - Navigation functional
	* 03 - Odometer functional
	* 04 - Light sensor correction functional
	* 05 - Console Inputs Functional
	* 06 - Ultrasonic localization complete
	* 07 - Functional search and localize system
	* 08 - Obstacle avoidance complete
	* 09 - Full Course pathing Complete
	* 10 - Finalized version (can do all trials)
	
	
## Version Setup
	*The version shall be written as such XX.XX.XX (ex: 04.02.13)
	*Every time a software milestone (obstacle breach) has been reached* the first number changes
	*Every time a new working implementation has been made the second number shall be changed
	*For small changes that do not add further functionality, the last number shall be changed

## Style Conventions
  * Commit messages must:
	* Start with the version number (refer to version setup)
	* Followed by Dash
    * Continue with the files modified (Use understandable abbreviations if needed): Ex: "Nav, OdoExce"
	* Followed once more by a dash
	* Continued by change explanation
    * Start change explanation with a verb in the passed tense
    * Begin with a capital letter
    * Have no period at the end
    * Be less than 80 characters
    * Be informative and meaningful
  * File/folder names must:
    * Be written in lowercase
    * Have words separated by underscores `_`
	
	
## General Workflow
1. Sync repository (or `git pull`)
2. Create a new branch  and checkout to that branch.
3. Make the modifications you wish to make.
4. Ascertain functionality
5. Commit and push to main
