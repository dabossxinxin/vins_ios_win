#pragma once

#include <cstdio>

// Use e.g. like this:
// VINS_INFO_STREAM("Info: this is a point: " << VINS::PointXYZ(1.0, 2.0, 3.0) << std::endl);
// VINS_ERROR_STREAM("Error: an Eigen vector: " << std::endl << Eigen::Vector3f(1.0, 2.0, 3.0) << std::endl);
#define VINS_LOG_STREAM(LEVEL, STREAM, CSTR, ATTR, FG, ARGS) if(console::isVerbosityLevelEnabled(console::LEVEL)) { fflush(stdout); console::change_text_color(CSTR, console::ATTR, console::FG); STREAM << ARGS; console::reset_text_color(CSTR); }
#define VINS_ALWAYS_STREAM(ARGS)  VINS_LOG_STREAM(L_ALWAYS,  std::cout, stdout, TT_RESET,  TT_WHITE,  ARGS)
#define VINS_ERROR_STREAM(ARGS)   VINS_LOG_STREAM(L_ERROR,   std::cerr, stderr, TT_BRIGHT, TT_RED,    ARGS)
#define VINS_WARN_STREAM(ARGS)    VINS_LOG_STREAM(L_WARN,    std::cerr, stderr, TT_BRIGHT, TT_YELLOW, ARGS)
#define VINS_INFO_STREAM(ARGS)    VINS_LOG_STREAM(L_INFO,    std::cout, stdout, TT_RESET,  TT_WHITE,  ARGS)
#define VINS_DEBUG_STREAM(ARGS)   VINS_LOG_STREAM(L_DEBUG,   std::cout, stdout, TT_RESET,  TT_GREEN,  ARGS)
#define VINS_VERBOSE_STREAM(ARGS) VINS_LOG_STREAM(L_VERBOSE, std::cout, stdout, TT_RESET,  TT_WHITE,  ARGS)

#define VINS_ALWAYS(...)  console::print (console::L_ALWAYS, __VA_ARGS__)
#define VINS_ERROR(...)   console::print (console::L_ERROR, __VA_ARGS__)
#define VINS_WARN(...)    console::print (console::L_WARN, __VA_ARGS__)
#define VINS_INFO(...)    console::print (console::L_INFO, __VA_ARGS__)
#define VINS_DEBUG(...)   console::print (console::L_DEBUG, __VA_ARGS__)
#define VINS_VERBOSE(...) console::print (console::L_VERBOSE, __VA_ARGS__)

#define VINS_ASSERT_ERROR_PRINT_CHECK(pred, msg) \
    do \
    { \
        if (!(pred)) \
        { \
            VINS_ERROR(msg); \
            VINS_ERROR("In File %s, in line %d\n" __FILE__, __LINE__); \
        } \
    } while (0)

#define VINS_ASSERT_ERROR_PRINT_RETURN(pred, msg, err) \
    do \
    { \
        VINS_ASSERT_ERROR_PRINT_CHECK(pred, msg); \
        if (!(pred)) return err; \
    } while (0)

namespace console
{
	enum TT_ATTIBUTES
	{
		TT_RESET = 0,
		TT_BRIGHT = 1,
		TT_DIM = 2,
		TT_UNDERLINE = 3,
		TT_BLINK = 4,
		TT_REVERSE = 7,
		TT_HIDDEN = 8
	};

	enum TT_COLORS
	{
		TT_BLACK,
		TT_RED,
		TT_GREEN,
		TT_YELLOW,
		TT_BLUE,
		TT_MAGENTA,
		TT_CYAN,
		TT_WHITE
	};

	enum VERBOSITY_LEVEL
	{
		L_ALWAYS,
		L_ERROR,
		L_WARN,
		L_INFO,
		L_DEBUG,
		L_VERBOSE
	};

	/** set the verbosity level */
	void
		setVerbosityLevel(VERBOSITY_LEVEL level);

	/** get the verbosity level. */
	VERBOSITY_LEVEL
		getVerbosityLevel();

	/** initialize verbosity level. */
	bool
		initVerbosityLevel();

	/** is verbosity level enabled? */
	bool
		isVerbosityLevelEnabled(VERBOSITY_LEVEL severity);

	/** \brief Enable or disable colored text output, overriding the default behavior.
	  *
	  * By default, colored output is enabled for interactive terminals or when the environment
	  * variable VINS_CLICOLOR_FORCE is set.
	  *
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param enable whether to emit color codes when calling any of the color related methods
	  */
	void
		enableColoredOutput(FILE *stream, bool enable);

	/** \brief Change the text color (on either stdout or stderr) with an attr:fg:bg
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param attribute the text attribute
	  * \param fg the foreground color
	  * \param bg the background color
	  */
	void
		change_text_color(FILE *stream, int attribute, int fg, int bg);

	/** \brief Change the text color (on either stdout or stderr) with an attr:fg
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param attribute the text attribute
	  * \param fg the foreground color
	  */
	void
		change_text_color(FILE *stream, int attribute, int fg);

	/** \brief Reset the text color (on either stdout or stderr) to its original state
	  * \param stream the output stream (stdout, stderr, etc)
	  */
	void
		reset_text_color(FILE *stream);

	/** \brief Print a message on stream with colors
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param attr the text attribute
	  * \param fg the foreground color
	  * \param format the message
	  */
	void
		print_color(FILE *stream, int attr, int fg, const char *format, ...);

	/** \brief Print an info message on stream with colors
	  * \param format the message
	  */
	void
		print_info(const char *format, ...);

	/** \brief Print an info message on stream with colors
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param format the message
	  */
	void
		print_info(FILE *stream, const char *format, ...);

	/** \brief Print a highlighted info message on stream with colors
	  * \param format the message
	  */
	void
		print_highlight(const char *format, ...);

	/** \brief Print a highlighted info message on stream with colors
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param format the message
	  */
	void
		print_highlight(FILE *stream, const char *format, ...);

	/** \brief Print an error message on stream with colors
	  * \param format the message
	  */
	void
		print_error(const char *format, ...);

	/** \brief Print an error message on stream with colors
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param format the message
	  */
	void
		print_error(FILE *stream, const char *format, ...);

	/** \brief Print a warning message on stream with colors
	  * \param format the message
	  */
	void
		print_warn(const char *format, ...);

	/** \brief Print a warning message on stream with colors
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param format the message
	  */
	void
		print_warn(FILE *stream, const char *format, ...);

	/** \brief Print a debug message on stream with colors
	  * \param format the message
	  */
	void
		print_debug(const char *format, ...);

	/** \brief Print a debug message on stream with colors
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param format the message
	  */
	void
		print_debug(FILE *stream, const char *format, ...);


	/** \brief Print a value message on stream with colors
	  * \param format the message
	  */
	void
		print_value(const char *format, ...);

	/** \brief Print a value message on stream with colors
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param format the message
	  */
	void
		print_value(FILE *stream, const char *format, ...);

	/** \brief Print a message on stream
	  * \param level the verbosity level
	  * \param stream the output stream (stdout, stderr, etc)
	  * \param format the message
	  */
	void
		print(VERBOSITY_LEVEL level, FILE *stream, const char *format, ...);

	/** \brief Print a message
	  * \param level the verbosity level
	  * \param format the message
	  */
	void
		print(VERBOSITY_LEVEL level, const char *format, ...);

	/** \brief Print a matrix
	  * \param data matrix data
	  * \param rows matrix rows
	  * \param cols matrix cols
	  * \param method 0 (major col) 1 (major row)
	  */
	void
		print_matrix(double* data, int rows, int cols, int method = 0);
}
