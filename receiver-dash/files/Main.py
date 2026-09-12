import dashboard
import webbrowser

if __name__ == "__main__":
    # Open the dashboard in the browser automatically
    webbrowser.open("http://127.0.0.1:8050/")

    # Run the Dash app
    dashboard.app.run(debug=False)