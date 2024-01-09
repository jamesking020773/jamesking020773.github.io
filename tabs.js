var navLinks;
var navPanels;

window.onload=function() {
    navLinks = document.getElementById("nav").getElementsByTagName("li");
	navPanels = document.getElementById("navPanels").getElementsByTagName("div");
    displayPanel(navLinks[0]);

    for (var i = 0; i < navLinks.length; i++) {
        navLinks[i].onclick = function() { 
			displayPanel(this); 
			return false;
		}
    }
}

function displayPanel(tabToActivate) {
    for (var i = 0; i < navLinks.length; i++) {
        if (navLinks[i] == tabToActivate) {
            navLinks[i].classList.add("active");
			navPanels[i].style.display = "block";
        } else {
        	navLinks[i].classList.remove("active");
			navPanels[i].style.display = "none";
        }
	}
}