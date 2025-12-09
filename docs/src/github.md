<style>
    img[src*='#left'] {
     float: left;
    }
    img[src*='#right'] {
        float: right;
    }
    img[src*='#center'] {
        display: block;
        margin: auto;
    }
</style>

# M2 Github ![MODAQ Github Organization](img/m2_logo.png#right)
This section will discuss the layout of the MODAQ 2 codebase on Github.

## Github Organization: MODAQ2
Because MODAQ 2 is designed to be modular, it was laid out using an organization in Github which can be found at this link:  <a href="https://github.com/MODAQ2" target="_blank">MODAQ2</a>. This enables the ability to clone or copy the necessary modules for your specific implementation of MODAQ 2. The main reason of taking this approach to software organization is to streamline the set up of a new system and streamline contributing to the open-source MODAQ 2 project.

## Building a MODAQ 2 System
It is recommended to utilize git and Github/Gitlab for building a MODAQ 2 system to efficiently track changes and maintain version control. We recommend initializing a new repo on your Github account and brining in the necessary modules for your project. You can also add any modules you may need that currently aren't available from the MODAQ2 organization. These are the steps to do this:

1. Login or make an account on github.com   
2. Make a new repository on your account - we recommend including the ROS .gitignore template
3. Start your development computer and navigate to the folder you want to keep your repo in. I.e. `cd ~`
4. Clone the repository you just created to your development computer using `git clone https://github.com/{your_account}/{your_repo}.git`
5. Move into your repository and create a src directory using `cd {your_repo}` and `mkdir src`
6. Clone the MODAQ2 repositories you need into the src folder with `git clone https://github.com/MODAQ2/{repos_you_need}.git`
7. To be able to push your build to your new github repository, you will need to delete the .git folder from the repos you have cloned. `cd m2_core` and `sudo rm .git -r`
8. You can now commit and push your code stack to github and it will be detached from the MODAQ2 organization
9. Please consider contributing to the MODAQ2 organization if you have changes the community can benefit from. This process is described below.

!!! note
    You will likely need to set up a personal access token to clone and push to your github repo. Visit this link to learn how to do that. <a href="https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens" target="_blank">Link</a>



## Contributing to MODAQ2 with Github Submodules
When contributing to the MODAQ2 organization, it is recommended to fork the MODAQ2 repos you are using and bring in them into your main repository as git submodules. This is essentially adding a git repo inside another git repo. The changes tracked in the submodule are actually relative to the repo of the submodule rather than the broader repo. Say you make a change to the labjack_t8_ros2 package that would be useful for everyone who uses labjacks, you can make a pull request on the labjack_t8_ros2 repo and your changes could help out all users of MODAQ 2. This meets our project requirement of maximizing modularity for MODAQ 2 users.

All of the current packages/modules for MODAQ 2 can be found on the <a href="https://github.com/orgs/MODAQ2/repositories" target="_blank">Github Organization</a>.

To fork the repositories you are using, follow these steps:

1. Login or make an account on github.com
2. Navigate in your browser to the MODAQ2 package you are wanting to contribute to
3. Click the fork button in the top right corner
4. Navigate to your personal fork of the repo. You can find it on your github account in your repositories
5. Get the URL of your personal fork. I.e. https://github.com/{your_account}/m2_core.git


The recommended process for getting started with submodules is to start a top level repo which will hold the submodules:
```bash
cd #go home
mkdir MODAQ2-Dev #this directory should be named based on your project name
cd MODAQ2-Dev
git init #init the top level repo
mkdir src #all ros2 packages should be in the src directory
cd src
git submodule add https://github.com/{your_account}/m2_core.git #this is the main module for MODAQ 2
git submodule add {Other MODAQ 2 packages/modules}
```

Now your fork of m2_core is a submodule in your MODAQ2-Dev repo. Changes you make to m2_core will be tracked locally to the m2_core forked repo.

To contribute your changes, push to your forked repo and then open a pull request to the repo on MODAQ2 on Github.com

More information on forking, submodules and pull request can be found in the [Useful Links](links.md#git-and-github)